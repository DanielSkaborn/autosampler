/*
 * autosampler - MIDI autosampler for Linux (ALSA)
 *
 * Plays MIDI notes to a hardware/software synth, records the audio coming
 * back, stores every note/velocity combination as a WAV file and builds an
 * SF2 (SoundFont 2) file that maps the whole sample set.
 *
 * Build:  make            (needs libasound2-dev)
 * Usage:  ./autosampler --help
 */
#define _GNU_SOURCE
#include <alsa/asoundlib.h>
#include <ctype.h>
#include <errno.h>
#include <getopt.h>
#include <limits.h>
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#define VERSION "1.0"
#define SF2_PAD_POINTS 46 /* zero points required after every sample */

/* ------------------------------------------------------------------ */
/* Configuration                                                       */
/* ------------------------------------------------------------------ */

enum { LAYOUT_FLAT = 0, LAYOUT_VEL, LAYOUT_NOTE };

typedef struct {
    char name[64];
    const char *outdir;
    const char *audio_dev;
    const char *midi_port;

    int note_lo, note_hi, note_step;
    int vel_lo, vel_hi, vel_step;
    int channel; /* 0..15 */
    int program; /* -1 = don't send */
    int bank;    /* -1 = don't send (CC0) */

    double length_ms;  /* time the note is held */
    double release_ms; /* time recorded after note-off */
    double preroll_ms; /* capture before note-on */
    double pause_ms;   /* silence between samples */

    unsigned rate;
    int channels; /* 1 or 2 */
    int bits;     /* WAV bit depth: 16 or 24 */

    double gain_db;
    int trim_silence;
    double threshold_db;
    double fade_ms;
    double latency_ms;

    int layout;
    int skip_existing;
    int sf2_only;
    int no_sf2;
    int dry_run;
    int extend; /* extend outermost key zones to 0 / 127 */
    double sf_release_ms;
} cfg_t;

typedef struct {
    int note, vel;
    char path[PATH_MAX];
    int present;
} sample_t;

static volatile sig_atomic_t g_stop = 0;

static void on_signal(int sig)
{
    (void)sig;
    g_stop = 1;
}

static void die(const char *msg)
{
    fprintf(stderr, "autosampler: %s\n", msg);
    exit(1);
}

/* ------------------------------------------------------------------ */
/* Small helpers                                                       */
/* ------------------------------------------------------------------ */

static void msleep(double ms)
{
    if (ms <= 0)
        return;
    struct timespec ts;
    ts.tv_sec = (time_t)(ms / 1000.0);
    ts.tv_nsec = (long)(fmod(ms, 1000.0) * 1e6);
    while (nanosleep(&ts, &ts) == -1 && errno == EINTR && !g_stop)
        ;
}

static size_t ms_to_frames(unsigned rate, double ms)
{
    return (size_t)llround(ms * rate / 1000.0);
}

static int mkdir_p(const char *path)
{
    char tmp[PATH_MAX];
    snprintf(tmp, sizeof tmp, "%s", path);
    for (char *p = tmp + 1; *p; p++) {
        if (*p == '/') {
            *p = 0;
            if (mkdir(tmp, 0777) < 0 && errno != EEXIST)
                return -1;
            *p = '/';
        }
    }
    if (mkdir(tmp, 0777) < 0 && errno != EEXIST)
        return -1;
    return 0;
}

static int file_exists(const char *path)
{
    struct stat st;
    return stat(path, &st) == 0 && S_ISREG(st.st_mode);
}

/* Note names: middle C = C4 = MIDI 60. fn=1 -> filename-safe ("Cs4"). */
static const char *const NOTE_DISP[12] = {"C", "C#", "D", "D#", "E", "F",
                                          "F#", "G", "G#", "A", "A#", "B"};
static const char *const NOTE_FILE[12] = {"C", "Cs", "D", "Ds", "E", "F",
                                          "Fs", "G", "Gs", "A", "As", "B"};

static void note_name(int note, int fn, char *out, size_t sz)
{
    snprintf(out, sz, "%s%d", (fn ? NOTE_FILE : NOTE_DISP)[note % 12],
             note / 12 - 1);
}

/* Accepts "60", "C4", "c#3", "Db2", "A-1" ... Returns -1 on error. */
static int parse_note(const char *s)
{
    char *end;
    long v = strtol(s, &end, 10);
    if (end != s && *end == 0)
        return (v >= 0 && v <= 127) ? (int)v : -1;

    int base;
    switch (toupper((unsigned char)s[0])) {
    case 'C': base = 0; break;
    case 'D': base = 2; break;
    case 'E': base = 4; break;
    case 'F': base = 5; break;
    case 'G': base = 7; break;
    case 'A': base = 9; break;
    case 'B': base = 11; break;
    default: return -1;
    }
    const char *p = s + 1;
    if (*p == '#') {
        base++;
        p++;
    } else if (*p == 'b') {
        base--;
        p++;
    }
    long oct = strtol(p, &end, 10);
    if (end == p || *end != 0)
        return -1;
    long n = (oct + 1) * 12 + base;
    return (n >= 0 && n <= 127) ? (int)n : -1;
}

static void sanitize_name(char *s)
{
    for (; *s; s++)
        if (!isalnum((unsigned char)*s) && *s != '-' && *s != '_')
            *s = '_';
}

/* ------------------------------------------------------------------ */
/* WAV I/O (little endian host assumed: x86, ARM Linux)                */
/* ------------------------------------------------------------------ */

static void put16(uint8_t *p, uint32_t v)
{
    p[0] = v & 255;
    p[1] = (v >> 8) & 255;
}
static void put32(uint8_t *p, uint32_t v)
{
    p[0] = v & 255;
    p[1] = (v >> 8) & 255;
    p[2] = (v >> 16) & 255;
    p[3] = (v >> 24) & 255;
}
static uint32_t get16(const uint8_t *p) { return p[0] | (p[1] << 8); }
static uint32_t get32(const uint8_t *p)
{
    return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* data: interleaved int32, left-justified (full scale = +/-2^31) */
static int wav_write(const char *path, const int32_t *data, size_t frames,
                     int ch, unsigned rate, int bits, int root_note)
{
    char tmp[PATH_MAX + 8];
    snprintf(tmp, sizeof tmp, "%s.tmp", path);

    size_t bps = bits / 8;
    size_t nsamp = frames * ch;
    size_t datasz = nsamp * bps;
    size_t pad = datasz & 1;
    uint8_t *buf = malloc(datasz ? datasz : 1);
    if (!buf)
        return -1;

    for (size_t i = 0; i < nsamp; i++) {
        int64_t x = data[i];
        if (bits == 16) {
            x = (x + 0x8000) >> 16;
            if (x > 32767) x = 32767;
            put16(buf + i * 2, (uint32_t)(int16_t)x);
        } else {
            x = (x + 0x80) >> 8;
            if (x > 8388607) x = 8388607;
            uint32_t u = (uint32_t)(int32_t)x;
            buf[i * 3] = u & 255;
            buf[i * 3 + 1] = (u >> 8) & 255;
            buf[i * 3 + 2] = (u >> 16) & 255;
        }
    }

    FILE *f = fopen(tmp, "wb");
    if (!f) {
        free(buf);
        return -1;
    }
    uint8_t h[12 + 24 + 44 + 8];
    uint8_t *p = h;
    memcpy(p, "RIFF", 4);
    put32(p + 4, (uint32_t)(4 + 24 + 44 + 8 + datasz + pad));
    memcpy(p + 8, "WAVE", 4);
    p += 12;
    memcpy(p, "fmt ", 4);
    put32(p + 4, 16);
    put16(p + 8, 1); /* PCM */
    put16(p + 10, ch);
    put32(p + 12, rate);
    put32(p + 16, (uint32_t)(rate * ch * bps));
    put16(p + 20, (uint32_t)(ch * bps));
    put16(p + 22, bits);
    p += 24;
    /* 'smpl' chunk carries the root key so samplers pick it up */
    memcpy(p, "smpl", 4);
    put32(p + 4, 36);
    memset(p + 8, 0, 36);
    put32(p + 8 + 8, (uint32_t)(1000000000.0 / rate)); /* sample period ns */
    put32(p + 8 + 12, root_note);                      /* MIDI unity note */
    p += 44;
    memcpy(p, "data", 4);
    put32(p + 4, (uint32_t)datasz);

    int ok = fwrite(h, sizeof h, 1, f) == 1 &&
             (datasz == 0 || fwrite(buf, datasz, 1, f) == 1) &&
             (!pad || fputc(0, f) != EOF);
    ok = (fclose(f) == 0) && ok;
    free(buf);
    if (!ok || rename(tmp, path) < 0) {
        unlink(tmp);
        return -1;
    }
    return 0;
}

/* Loads PCM wav (16/24/32 bit) into left-justified int32. */
static int wav_load(const char *path, int32_t **out, size_t *frames, int *ch,
                    unsigned *rate)
{
    FILE *f = fopen(path, "rb");
    if (!f)
        return -1;
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz < 44) {
        fclose(f);
        return -1;
    }
    uint8_t *b = malloc(sz);
    if (!b || fread(b, 1, sz, f) != (size_t)sz) {
        free(b);
        fclose(f);
        return -1;
    }
    fclose(f);
    if (memcmp(b, "RIFF", 4) || memcmp(b + 8, "WAVE", 4)) {
        free(b);
        return -1;
    }

    int bits = 0, chn = 0, fmt = 0;
    unsigned sr = 0;
    const uint8_t *data = NULL;
    size_t dsz = 0;
    long pos = 12;
    while (pos + 8 <= sz) {
        uint32_t csz = get32(b + pos + 4);
        const uint8_t *c = b + pos + 8;
        if (!memcmp(b + pos, "fmt ", 4) && csz >= 16 && pos + 8 + 16 <= sz) {
            fmt = get16(c);
            chn = get16(c + 2);
            sr = get32(c + 4);
            bits = get16(c + 14);
        } else if (!memcmp(b + pos, "data", 4)) {
            data = c;
            dsz = csz;
            if (pos + 8 + (long)dsz > sz)
                dsz = sz - pos - 8;
            break;
        }
        pos += 8 + (long)csz + (csz & 1);
    }
    if (!data || (fmt != 1 && fmt != 0xFFFE) || chn < 1 || chn > 2 ||
        (bits != 16 && bits != 24 && bits != 32)) {
        free(b);
        return -1;
    }
    size_t bps = bits / 8;
    size_t n = dsz / bps / chn;
    int32_t *o = malloc((n * chn + 1) * sizeof(int32_t));
    if (!o) {
        free(b);
        return -1;
    }
    for (size_t i = 0; i < n * chn; i++) {
        const uint8_t *s = data + i * bps;
        if (bits == 16)
            o[i] = (int32_t)((uint32_t)get16(s) << 16);
        else if (bits == 24)
            o[i] = (int32_t)(((uint32_t)s[0] << 8) | ((uint32_t)s[1] << 16) |
                             ((uint32_t)s[2] << 24));
        else
            o[i] = (int32_t)get32(s);
    }
    free(b);
    *out = o;
    *frames = n;
    *ch = chn;
    *rate = sr;
    return 0;
}

/* ------------------------------------------------------------------ */
/* MIDI output (ALSA sequencer)                                        */
/* ------------------------------------------------------------------ */

typedef struct {
    snd_seq_t *seq;
    int port;
} midi_t;

static int midi_send(midi_t *m, snd_seq_event_t *ev)
{
    snd_seq_ev_set_source(ev, m->port);
    snd_seq_ev_set_subs(ev);
    snd_seq_ev_set_direct(ev);
    int r = snd_seq_event_output_direct(m->seq, ev);
    return r < 0 ? r : 0;
}

static void midi_note(midi_t *m, int ch, int note, int vel, int on)
{
    snd_seq_event_t ev;
    snd_seq_ev_clear(&ev);
    if (on)
        snd_seq_ev_set_noteon(&ev, ch, note, vel);
    else
        snd_seq_ev_set_noteoff(&ev, ch, note, 0);
    midi_send(m, &ev);
}

static void midi_cc(midi_t *m, int ch, int cc, int val)
{
    snd_seq_event_t ev;
    snd_seq_ev_clear(&ev);
    snd_seq_ev_set_controller(&ev, ch, cc, val);
    midi_send(m, &ev);
}

static void midi_silence(midi_t *m, int ch)
{
    midi_cc(m, ch, 123, 0); /* all notes off */
    midi_cc(m, ch, 120, 0); /* all sound off */
}

static int midi_open(midi_t *m, const char *dest)
{
    if (snd_seq_open(&m->seq, "default", SND_SEQ_OPEN_OUTPUT, 0) < 0) {
        fprintf(stderr, "Cannot open ALSA sequencer\n");
        return -1;
    }
    snd_seq_set_client_name(m->seq, "autosampler");
    m->port = snd_seq_create_simple_port(
        m->seq, "out", SND_SEQ_PORT_CAP_READ | SND_SEQ_PORT_CAP_SUBS_READ,
        SND_SEQ_PORT_TYPE_MIDI_GENERIC | SND_SEQ_PORT_TYPE_APPLICATION);
    if (m->port < 0) {
        fprintf(stderr, "Cannot create MIDI port\n");
        return -1;
    }
    snd_seq_addr_t addr;
    if (snd_seq_parse_address(m->seq, &addr, dest) < 0) {
        fprintf(stderr, "Unknown MIDI port '%s' (see --list-midi)\n", dest);
        return -1;
    }
    if (snd_seq_connect_to(m->seq, m->port, addr.client, addr.port) < 0) {
        fprintf(stderr, "Cannot connect to MIDI port %d:%d\n", addr.client,
                addr.port);
        return -1;
    }
    return 0;
}

static int list_midi(void)
{
    snd_seq_t *seq;
    if (snd_seq_open(&seq, "default", SND_SEQ_OPEN_OUTPUT, 0) < 0) {
        fprintf(stderr, "Cannot open ALSA sequencer\n");
        return 1;
    }
    snd_seq_client_info_t *ci;
    snd_seq_port_info_t *pi;
    snd_seq_client_info_alloca(&ci);
    snd_seq_port_info_alloca(&pi);
    snd_seq_client_info_set_client(ci, -1);
    printf("MIDI output ports (use with --midi-port):\n");
    while (snd_seq_query_next_client(seq, ci) >= 0) {
        int cl = snd_seq_client_info_get_client(ci);
        snd_seq_port_info_set_client(pi, cl);
        snd_seq_port_info_set_port(pi, -1);
        while (snd_seq_query_next_port(seq, pi) >= 0) {
            unsigned caps = snd_seq_port_info_get_capability(pi);
            if ((caps & (SND_SEQ_PORT_CAP_WRITE | SND_SEQ_PORT_CAP_SUBS_WRITE)) !=
                (SND_SEQ_PORT_CAP_WRITE | SND_SEQ_PORT_CAP_SUBS_WRITE))
                continue;
            if (caps & SND_SEQ_PORT_CAP_NO_EXPORT)
                continue;
            printf("  %3d:%-2d  %s / %s\n", cl, snd_seq_port_info_get_port(pi),
                   snd_seq_client_info_get_name(ci),
                   snd_seq_port_info_get_name(pi));
        }
    }
    snd_seq_close(seq);
    return 0;
}

static int list_audio(void)
{
    void **hints;
    if (snd_device_name_hint(-1, "pcm", &hints) < 0) {
        fprintf(stderr, "Cannot enumerate ALSA devices\n");
        return 1;
    }
    printf("Audio capture devices (use with --audio-device):\n");
    for (void **h = hints; *h; h++) {
        char *name = snd_device_name_get_hint(*h, "NAME");
        char *desc = snd_device_name_get_hint(*h, "DESC");
        char *ioid = snd_device_name_get_hint(*h, "IOID");
        if (name && (!ioid || !strcmp(ioid, "Input"))) {
            printf("  %s\n", name);
            if (desc) {
                for (char *p = desc; *p; p++)
                    if (*p == '\n')
                        *p = ' ';
                printf("      %s\n", desc);
            }
        }
        free(name);
        free(desc);
        free(ioid);
    }
    snd_device_name_free_hint(hints);
    return 0;
}

/* ------------------------------------------------------------------ */
/* Audio capture (ALSA PCM)                                            */
/* ------------------------------------------------------------------ */

typedef struct {
    snd_pcm_t *pcm;
    snd_pcm_format_t fmt;
    unsigned ch;
    snd_pcm_uframes_t chunk;
    int32_t *raw;
} capture_t;

static int cap_open(capture_t *c, const char *dev, unsigned rate, unsigned ch)
{
    int err = snd_pcm_open(&c->pcm, dev, SND_PCM_STREAM_CAPTURE, 0);
    if (err < 0) {
        fprintf(stderr, "Cannot open audio device '%s': %s\n", dev,
                snd_strerror(err));
        return -1;
    }
    snd_pcm_hw_params_t *hw;
    snd_pcm_hw_params_alloca(&hw);
    snd_pcm_hw_params_any(c->pcm, hw);
    if (snd_pcm_hw_params_set_access(c->pcm, hw,
                                     SND_PCM_ACCESS_RW_INTERLEAVED) < 0) {
        fprintf(stderr, "Device does not support interleaved access\n");
        return -1;
    }
    static const snd_pcm_format_t fmts[] = {SND_PCM_FORMAT_S32_LE,
                                            SND_PCM_FORMAT_S24_LE,
                                            SND_PCM_FORMAT_S16_LE};
    int ok = 0;
    for (size_t i = 0; i < sizeof fmts / sizeof *fmts; i++) {
        if (snd_pcm_hw_params_set_format(c->pcm, hw, fmts[i]) == 0) {
            c->fmt = fmts[i];
            ok = 1;
            break;
        }
    }
    if (!ok) {
        fprintf(stderr, "No supported sample format (S32/S24/S16 LE). "
                        "Try a plughw: device.\n");
        return -1;
    }
    if (snd_pcm_hw_params_set_channels(c->pcm, hw, ch) < 0) {
        fprintf(stderr, "Device does not support %u channel(s)\n", ch);
        return -1;
    }
    unsigned r = rate;
    snd_pcm_hw_params_set_rate_near(c->pcm, hw, &r, 0);
    if (r != rate) {
        fprintf(stderr, "Device cannot do %u Hz (offers %u Hz). "
                        "Try a plughw: device.\n", rate, r);
        return -1;
    }
    snd_pcm_uframes_t period = rate / 100; /* ~10 ms */
    snd_pcm_uframes_t buffer = period * 64;
    snd_pcm_hw_params_set_period_size_near(c->pcm, hw, &period, 0);
    snd_pcm_hw_params_set_buffer_size_near(c->pcm, hw, &buffer);
    if ((err = snd_pcm_hw_params(c->pcm, hw)) < 0) {
        fprintf(stderr, "Cannot set audio parameters: %s\n", snd_strerror(err));
        return -1;
    }
    c->ch = ch;
    c->chunk = period;
    c->raw = malloc(c->chunk * ch * sizeof(int32_t));
    return c->raw ? 0 : -1;
}

/* Reads exactly `frames` frames (blocking) into dst as left-justified int32. */
static int cap_read(capture_t *c, int32_t *dst, size_t frames)
{
    while (frames) {
        if (g_stop)
            return -EINTR;
        size_t want = frames < c->chunk ? frames : c->chunk;
        snd_pcm_sframes_t n = snd_pcm_readi(c->pcm, c->raw, want);
        if (n < 0) {
            if (n == -EINTR && g_stop)
                return -EINTR;
            return (int)n; /* -EPIPE = overrun: caller retries the sample */
        }
        size_t cnt = (size_t)n * c->ch;
        switch (c->fmt) {
        case SND_PCM_FORMAT_S32_LE:
            memcpy(dst, c->raw, cnt * sizeof(int32_t));
            break;
        case SND_PCM_FORMAT_S24_LE:
            for (size_t i = 0; i < cnt; i++)
                dst[i] = (int32_t)((uint32_t)c->raw[i] << 8);
            break;
        default: /* S16_LE */
            for (size_t i = 0; i < cnt; i++)
                dst[i] = (int32_t)((uint32_t)(uint16_t)((int16_t *)c->raw)[i]
                                   << 16);
            break;
        }
        dst += cnt;
        frames -= (size_t)n;
    }
    return 0;
}

/*
 * Records one note. buf must hold (pre+len+rel)*channels int32.
 * Capture starts, `pre` frames are recorded, note-on is sent, the note is held
 * for `len` frames, note-off is sent, and `rel` frames of release are recorded.
 * Timing is derived from the audio clock, not from wall-clock sleeps.
 */
static int record_sample(const cfg_t *c, capture_t *cap, midi_t *m, int note,
                         int vel, int32_t *buf, size_t pre, size_t len,
                         size_t rel)
{
    int ch = c->channels, r;
    int32_t *p = buf;

    snd_pcm_drop(cap->pcm);
    snd_pcm_prepare(cap->pcm);
    if (snd_pcm_start(cap->pcm) < 0)
        return -EIO;

    r = cap_read(cap, p, pre);
    if (r < 0)
        goto out;
    p += pre * ch;

    midi_note(m, c->channel, note, vel, 1);
    r = cap_read(cap, p, len);
    midi_note(m, c->channel, note, vel, 0);
    if (r < 0)
        goto out;
    p += len * ch;

    r = cap_read(cap, p, rel);
out:
    snd_pcm_drop(cap->pcm);
    midi_silence(m, c->channel);
    return r;
}

/* ------------------------------------------------------------------ */
/* Post processing                                                     */
/* ------------------------------------------------------------------ */

static double to_db(int64_t peak)
{
    if (peak <= 0)
        return -999.0;
    return 20.0 * log10((double)peak / 2147483648.0);
}

/* Applies latency cut, gain, silence trim, fade-out. Returns peak. */
static int64_t process_sample(const cfg_t *c, int32_t **pp, size_t *pn,
                              int *silent)
{
    int ch = c->channels;
    int32_t *p = *pp;
    size_t n = *pn;
    *silent = 0;

    size_t lat = ms_to_frames(c->rate, c->latency_ms);
    if (lat && lat < n) {
        p += lat * ch;
        n -= lat;
    }

    if (c->gain_db != 0.0) {
        double g = pow(10.0, c->gain_db / 20.0);
        for (size_t i = 0; i < n * ch; i++) {
            double v = p[i] * g;
            p[i] = v > 2147483647.0 ? INT32_MAX
                                    : (v < -2147483648.0 ? INT32_MIN : (int32_t)v);
        }
    }

    int64_t thr = (int64_t)(2147483647.0 * pow(10.0, c->threshold_db / 20.0));
    int64_t peak = 0;
    size_t first = n, last = 0;
    for (size_t i = 0; i < n; i++) {
        for (int k = 0; k < ch; k++) {
            int64_t a = p[i * ch + k];
            if (a < 0) a = -a;
            if (a > peak) peak = a;
            if (a > thr) {
                if (i < first) first = i;
                last = i;
            }
        }
    }
    if (first >= n) {
        *silent = 1; /* never exceeded threshold: leave untouched */
    } else if (c->trim_silence) {
        p += first * ch;
        n = last - first + 1;
    }

    size_t fade = ms_to_frames(c->rate, c->fade_ms);
    if (fade > n)
        fade = n;
    for (size_t i = 0; i < fade; i++) {
        double g = 1.0 - (double)(i + 1) / (double)fade;
        for (int k = 0; k < ch; k++) {
            size_t idx = (n - fade + i) * ch + k;
            p[idx] = (int32_t)(p[idx] * g);
        }
    }

    *pp = p;
    *pn = n;
    return peak;
}

/* ------------------------------------------------------------------ */
/* SoundFont 2 writer                                                  */
/* ------------------------------------------------------------------ */

typedef struct {
    uint8_t *p;
    size_t n, cap;
} bbuf_t;

static void bb_put(bbuf_t *b, const void *d, size_t n)
{
    if (b->n + n > b->cap) {
        b->cap = (b->n + n) * 2 + 256;
        b->p = realloc(b->p, b->cap);
        if (!b->p)
            die("out of memory");
    }
    memcpy(b->p + b->n, d, n);
    b->n += n;
}
static void bb_u16(bbuf_t *b, uint32_t v)
{
    uint8_t t[2];
    put16(t, v);
    bb_put(b, t, 2);
}
static void bb_u32(bbuf_t *b, uint32_t v)
{
    uint8_t t[4];
    put32(t, v);
    bb_put(b, t, 4);
}
static void bb_str(bbuf_t *b, const char *s, size_t field)
{
    char t[64] = {0};
    snprintf(t, field < sizeof t ? field : sizeof t, "%s", s);
    bb_put(b, t, field);
}
static void bb_zero(bbuf_t *b, size_t n)
{
    uint8_t z[64] = {0};
    while (n) {
        size_t k = n < sizeof z ? n : sizeof z;
        bb_put(b, z, k);
        n -= k;
    }
}
/* append a sub-chunk (id,size,data[,pad]) to b */
static void bb_chunk(bbuf_t *b, const char *id, const bbuf_t *data)
{
    bb_put(b, id, 4);
    bb_u32(b, (uint32_t)data->n);
    bb_put(b, data->p, data->n);
    if (data->n & 1)
        bb_zero(b, 1);
}

enum { G_PAN = 17, G_RELEASE = 38, G_INSTRUMENT = 41, G_KEYRANGE = 43,
       G_VELRANGE = 44, G_SAMPLEID = 53, G_ROOTKEY = 58 };

static void gen(bbuf_t *b, int op, int amount)
{
    bb_u16(b, op);
    bb_u16(b, (uint32_t)(amount & 0xFFFF));
}

static int build_sf2(const cfg_t *c, sample_t *s, int ns, const char *path)
{
    int *pi = malloc(sizeof(int) * (ns ? ns : 1));
    size_t *pf = malloc(sizeof(size_t) * (ns ? ns : 1));
    int np = 0, chn = 0;
    unsigned rate = 0;
    if (!pi || !pf)
        die("out of memory");

    /* pass 1: which samples exist, how long are they */
    for (int i = 0; i < ns; i++) {
        if (!s[i].present)
            continue;
        int32_t *d;
        size_t n;
        int ch;
        unsigned sr;
        if (wav_load(s[i].path, &d, &n, &ch, &sr) < 0) {
            fprintf(stderr, "SF2: cannot read %s, skipping\n", s[i].path);
            continue;
        }
        free(d);
        if (np == 0) {
            chn = ch;
            rate = sr;
        } else if (ch != chn || sr != rate) {
            fprintf(stderr, "SF2: %s has different format, skipping\n",
                    s[i].path);
            continue;
        }
        if (n < 2) {
            fprintf(stderr, "SF2: %s is empty, skipping\n", s[i].path);
            continue;
        }
        pi[np] = i;
        pf[np] = n;
        np++;
    }
    if (np == 0) {
        fprintf(stderr, "SF2: no samples found, nothing to do\n");
        free(pi);
        free(pf);
        return -1;
    }

    /* total sample-data points */
    uint64_t points = 0;
    for (int k = 0; k < np; k++)
        points += (uint64_t)chn * (pf[k] + SF2_PAD_POINTS);
    if (points * 2 > 0xFFFFFF00u) {
        fprintf(stderr, "SF2: sample data exceeds the 4 GB format limit\n");
        free(pi);
        free(pf);
        return -1;
    }

    /* --- key / velocity zone computation --- */
    int *klo = malloc(sizeof(int) * np), *khi = malloc(sizeof(int) * np);
    int *vlo = malloc(sizeof(int) * np), *vhi = malloc(sizeof(int) * np);
    int *gs = malloc(sizeof(int) * np), *ge = malloc(sizeof(int) * np);
    if (!klo || !khi || !vlo || !vhi || !gs || !ge)
        die("out of memory");
    int ng = 0;
    for (int k = 0; k < np;) {
        int note = s[pi[k]].note, e = k;
        while (e + 1 < np && s[pi[e + 1]].note == note)
            e++;
        gs[ng] = k;
        ge[ng] = e;
        ng++;
        k = e + 1;
    }
    for (int g = 0; g < ng; g++) {
        int n = s[pi[gs[g]]].note;
        int pn = g > 0 ? s[pi[gs[g - 1]]].note : n;
        int nn = g < ng - 1 ? s[pi[gs[g + 1]]].note : n;
        int lo, hi;
        if (g == 0)
            lo = c->extend ? 0 : (ng > 1 ? n - (nn - n) / 2 : n);
        else
            lo = (pn + n) / 2 + 1;
        if (g == ng - 1)
            hi = c->extend ? 127 : (ng > 1 ? n + (n - pn) / 2 : n);
        else
            hi = (n + nn) / 2;
        if (lo < 0) lo = 0;
        if (hi > 127) hi = 127;
        for (int k = gs[g]; k <= ge[g]; k++) {
            klo[k] = lo;
            khi[k] = hi;
            int v = s[pi[k]].vel;
            int pv = k > gs[g] ? s[pi[k - 1]].vel : v;
            int nv = k < ge[g] ? s[pi[k + 1]].vel : v;
            vlo[k] = (k == gs[g]) ? 1 : (pv + v) / 2 + 1;
            vhi[k] = (k == ge[g]) ? 127 : (v + nv) / 2;
        }
    }

    double rel_s = c->sf_release_ms / 1000.0;
    if (rel_s < 0.001) rel_s = 0.001;
    int rel_tc = (int)lround(1200.0 * log2(rel_s));

    int nsh = np * chn; /* number of SF2 sample headers == zones */

    /* --- INFO list --- */
    bbuf_t info = {0}, tmp = {0};
    bb_put(&info, "INFO", 4);
    bb_u16(&tmp, 2); bb_u16(&tmp, 1);
    bb_chunk(&info, "ifil", &tmp);
    tmp.n = 0; bb_str(&tmp, "EMU8000", 8);
    bb_chunk(&info, "isng", &tmp);
    tmp.n = 0; bb_str(&tmp, c->name, (strlen(c->name) + 2) & ~1u);
    bb_chunk(&info, "INAM", &tmp);
    tmp.n = 0; bb_str(&tmp, "autosampler " VERSION, 16);
    bb_chunk(&info, "ISFT", &tmp);

    /* --- pdta --- */
    bbuf_t pdta = {0}, ch_ = {0};
    bb_put(&pdta, "pdta", 4);

    /* phdr */
    bb_str(&ch_, c->name, 20); bb_u16(&ch_, 0); bb_u16(&ch_, 0);
    bb_u16(&ch_, 0); bb_u32(&ch_, 0); bb_u32(&ch_, 0); bb_u32(&ch_, 0);
    bb_str(&ch_, "EOP", 20); bb_u16(&ch_, 0); bb_u16(&ch_, 0);
    bb_u16(&ch_, 1); bb_u32(&ch_, 0); bb_u32(&ch_, 0); bb_u32(&ch_, 0);
    bb_chunk(&pdta, "phdr", &ch_);
    /* pbag */
    ch_.n = 0;
    bb_u16(&ch_, 0); bb_u16(&ch_, 0);
    bb_u16(&ch_, 1); bb_u16(&ch_, 0);
    bb_chunk(&pdta, "pbag", &ch_);
    /* pmod (terminal only) */
    ch_.n = 0; bb_zero(&ch_, 10);
    bb_chunk(&pdta, "pmod", &ch_);
    /* pgen */
    ch_.n = 0;
    gen(&ch_, G_INSTRUMENT, 0);
    bb_zero(&ch_, 4);
    bb_chunk(&pdta, "pgen", &ch_);
    /* inst */
    ch_.n = 0;
    bb_str(&ch_, c->name, 20); bb_u16(&ch_, 0);
    bb_str(&ch_, "EOI", 20); bb_u16(&ch_, nsh);
    bb_chunk(&pdta, "inst", &ch_);
    /* ibag + igen */
    bbuf_t ibag = {0}, igen = {0};
    int gens_per_zone = chn == 2 ? 6 : 5;
    for (int k = 0; k < np; k++) {
        for (int j = 0; j < chn; j++) {
            bb_u16(&ibag, (uint32_t)(igen.n / 4));
            bb_u16(&ibag, 0);
            gen(&igen, G_KEYRANGE, klo[k] | (khi[k] << 8));
            gen(&igen, G_VELRANGE, vlo[k] | (vhi[k] << 8));
            if (chn == 2)
                gen(&igen, G_PAN, j == 0 ? -500 : 500);
            gen(&igen, G_RELEASE, rel_tc);
            gen(&igen, G_ROOTKEY, s[pi[k]].note);
            gen(&igen, G_SAMPLEID, k * chn + j);
        }
    }
    (void)gens_per_zone;
    bb_u16(&ibag, (uint32_t)(igen.n / 4));
    bb_u16(&ibag, 0);
    bb_zero(&igen, 4);
    bb_chunk(&pdta, "ibag", &ibag);
    /* imod (terminal only) */
    ch_.n = 0; bb_zero(&ch_, 10);
    bb_chunk(&pdta, "imod", &ch_);
    bb_chunk(&pdta, "igen", &igen);
    /* shdr */
    ch_.n = 0;
    uint32_t off = 0;
    for (int k = 0; k < np; k++) {
        char nm[16], label[32];
        note_name(s[pi[k]].note, 0, nm, sizeof nm);
        for (int j = 0; j < chn; j++) {
            snprintf(label, sizeof label, "%s v%03d%s", nm, s[pi[k]].vel,
                     chn == 2 ? (j == 0 ? " L" : " R") : "");
            uint32_t start = off, end = off + (uint32_t)pf[k];
            bb_str(&ch_, label, 20);
            bb_u32(&ch_, start);
            bb_u32(&ch_, end);
            bb_u32(&ch_, start);
            bb_u32(&ch_, end);
            bb_u32(&ch_, rate);
            bb_put(&ch_, &(uint8_t){(uint8_t)s[pi[k]].note}, 1);
            bb_put(&ch_, &(uint8_t){0}, 1);
            if (chn == 2) {
                bb_u16(&ch_, k * 2 + (j == 0 ? 1 : 0));
                bb_u16(&ch_, j == 0 ? 4 : 2); /* left=4, right=2 */
            } else {
                bb_u16(&ch_, 0);
                bb_u16(&ch_, 1);
            }
            off = end + SF2_PAD_POINTS;
        }
    }
    bb_str(&ch_, "EOS", 20);
    bb_zero(&ch_, 26);
    bb_chunk(&pdta, "shdr", &ch_);

    /* --- write file --- */
    uint32_t smpl_bytes = (uint32_t)(points * 2);
    uint32_t sdta_size = 4 + 8 + smpl_bytes;
    uint32_t riff_size = 4 + (8 + (uint32_t)info.n) + (8 + sdta_size) +
                         (8 + (uint32_t)pdta.n);

    FILE *f = fopen(path, "wb");
    if (!f) {
        perror(path);
        return -1;
    }
    uint8_t h[12];
    memcpy(h, "RIFF", 4); put32(h + 4, riff_size); memcpy(h + 8, "sfbk", 4);
    fwrite(h, 12, 1, f);
    memcpy(h, "LIST", 4); put32(h + 4, (uint32_t)info.n); fwrite(h, 8, 1, f);
    fwrite(info.p, info.n, 1, f);
    memcpy(h, "LIST", 4); put32(h + 4, sdta_size); fwrite(h, 8, 1, f);
    fwrite("sdta", 4, 1, f);
    memcpy(h, "smpl", 4); put32(h + 4, smpl_bytes); fwrite(h, 8, 1, f);

    /* pass 2: stream the 16-bit sample data */
    int rc = 0;
    uint8_t zeros[SF2_PAD_POINTS * 2] = {0};
    for (int k = 0; k < np && !rc; k++) {
        int32_t *d;
        size_t n;
        int ch;
        unsigned sr;
        if (wav_load(s[pi[k]].path, &d, &n, &ch, &sr) < 0 || n < pf[k]) {
            fprintf(stderr, "SF2: lost %s while writing\n", s[pi[k]].path);
            rc = -1;
            break;
        }
        uint8_t *out = malloc(pf[k] * 2);
        if (!out)
            die("out of memory");
        for (int j = 0; j < chn; j++) {
            for (size_t i = 0; i < pf[k]; i++) {
                int64_t x = ((int64_t)d[i * chn + j] + 0x8000) >> 16;
                if (x > 32767) x = 32767;
                put16(out + i * 2, (uint32_t)(int16_t)x);
            }
            fwrite(out, 2, pf[k], f);
            fwrite(zeros, sizeof zeros, 1, f);
        }
        free(out);
        free(d);
    }

    memcpy(h, "LIST", 4); put32(h + 4, (uint32_t)pdta.n); fwrite(h, 8, 1, f);
    fwrite(pdta.p, pdta.n, 1, f);
    if (fclose(f) != 0)
        rc = -1;

    printf("SoundFont: %s  (%d samples, %d zones, %s, %u Hz)\n", path, np,
           nsh, chn == 2 ? "stereo" : "mono", rate);

    free(info.p); free(tmp.p); free(pdta.p); free(ch_.p);
    free(ibag.p); free(igen.p);
    free(klo); free(khi); free(vlo); free(vhi); free(gs); free(ge);
    free(pi); free(pf);
    return rc;
}

/* ------------------------------------------------------------------ */
/* Sample plan                                                         */
/* ------------------------------------------------------------------ */

static sample_t *build_plan(const cfg_t *c, int *count)
{
    int nn = (c->note_hi - c->note_lo) / c->note_step + 1;
    int nv = (c->vel_hi - c->vel_lo) / c->vel_step + 1;
    sample_t *s = calloc((size_t)nn * nv, sizeof *s);
    if (!s)
        die("out of memory");
    int k = 0;
    char base[PATH_MAX], dir[PATH_MAX], nm[16];
    snprintf(base, sizeof base, "%s/%s", c->outdir, c->name);
    for (int n = c->note_lo; n <= c->note_hi; n += c->note_step) {
        for (int v = c->vel_lo; v <= c->vel_hi; v += c->vel_step) {
            note_name(n, 1, nm, sizeof nm);
            switch (c->layout) {
            case LAYOUT_VEL:
                snprintf(dir, sizeof dir, "%s/v%03d", base, v);
                break;
            case LAYOUT_NOTE:
                snprintf(dir, sizeof dir, "%s/%03d_%s", base, n, nm);
                break;
            default:
                snprintf(dir, sizeof dir, "%s", base);
            }
            s[k].note = n;
            s[k].vel = v;
            snprintf(s[k].path, sizeof s[k].path, "%s/%s_%03d_%s_v%03d.wav",
                     dir, c->name, n, nm, v);
            k++;
        }
    }
    *count = k;
    return s;
}

static void write_manifest(const cfg_t *c, const sample_t *s, int ns)
{
    char path[PATH_MAX], nm[16];
    snprintf(path, sizeof path, "%s/%s/%s_manifest.csv", c->outdir, c->name,
             c->name);
    FILE *f = fopen(path, "w");
    if (!f)
        return;
    fprintf(f, "midi_note,note_name,velocity,file\n");
    for (int i = 0; i < ns; i++) {
        if (!s[i].present)
            continue;
        note_name(s[i].note, 0, nm, sizeof nm);
        const char *rel = s[i].path + strlen(c->outdir);
        while (*rel == '/')
            rel++;
        fprintf(f, "%d,%s,%d,%s\n", s[i].note, nm, s[i].vel, rel);
    }
    fclose(f);
}

/* ------------------------------------------------------------------ */
/* Command line                                                        */
/* ------------------------------------------------------------------ */

static void usage(void)
{
    puts(
"autosampler " VERSION " - MIDI autosampler: records a synth into WAV files + SF2\n"
"\n"
"Usage: autosampler [options]\n"
"\n"
"Devices\n"
"  --list-midi               list MIDI output ports and exit\n"
"  --list-audio              list ALSA capture devices and exit\n"
"  -m, --midi-port PORT      MIDI destination, e.g. 128:0 or a client name\n"
"  -a, --audio-device DEV    ALSA capture device (default: \"default\")\n"
"  --channel N               MIDI channel 1-16 (default 1)\n"
"  --program N               send program change 0-127 before sampling\n"
"  --bank N                  send bank select MSB (CC0) 0-127 before sampling\n"
"\n"
"Note / velocity grid   (notes: number or name; middle C = C4 = 60)\n"
"  --note-min N              lowest note              (default C2)\n"
"  --note-max N              highest note             (default C6)\n"
"  --note-step N             semitones between samples (default 3)\n"
"  --vel-min N               lowest velocity 1-127    (default 100)\n"
"  --vel-max N               highest velocity 1-127   (default 100)\n"
"  --vel-step N              velocity increment       (default 32)\n"
"\n"
"Timing (milliseconds)\n"
"  -l, --length MS           how long each note is held       (default 2000)\n"
"  -r, --release MS          release tail recorded after note-off (default 1000)\n"
"  --preroll MS              capture time before note-on      (default 100)\n"
"  --pause MS                wait between samples             (default 300)\n"
"\n"
"Audio format\n"
"  --rate HZ                 sample rate (default 48000)\n"
"  --channels N              1 = mono, 2 = stereo (default 2)\n"
"  --bits N                  WAV bit depth 16 or 24 (default 24); SF2 is always 16\n"
"  --gain DB                 apply gain to recordings (default 0)\n"
"  --trim-silence            trim leading/trailing silence (start at the onset)\n"
"  --threshold DB            silence threshold (default -70)\n"
"  --fade MS                 fade-out at end of each sample (default 5)\n"
"  --latency MS              discard this much from the start (round-trip latency)\n"
"\n"
"Output\n"
"  -n, --name NAME           instrument name (default \"Sampled\")\n"
"  -o, --outdir DIR          output directory (default \".\")\n"
"                            files go to DIR/NAME/NAME_<note>_<name>_v<vel>.wav\n"
"  --layout flat|vel|note    flat: all files in one folder (default)\n"
"                            vel : one sub-folder per velocity layer\n"
"                            note: one sub-folder per note\n"
"  --skip-existing           don't re-record samples whose WAV already exists\n"
"  --sf2-only                don't record; (re)build SF2 from existing WAVs\n"
"  --no-sf2                  don't generate the SoundFont\n"
"  --sf-release MS           SF2 volume-envelope release (default 300)\n"
"  --no-extend               don't stretch the outer key zones to 0 / 127\n"
"  --dry-run                 print the sample plan and exit\n"
"  -h, --help                show this help\n"
"\n"
"Example:\n"
"  autosampler -m 128:0 -a hw:1,0 -n MyPiano --note-min C1 --note-max C7 \\\n"
"      --note-step 3 --vel-min 32 --vel-max 127 --vel-step 32 \\\n"
"      -l 4000 -r 1500 --trim-silence --layout vel");
}

enum {
    OPT_LIST_MIDI = 256, OPT_LIST_AUDIO, OPT_CHANNEL, OPT_PROGRAM, OPT_BANK,
    OPT_NOTE_MIN, OPT_NOTE_MAX, OPT_NOTE_STEP, OPT_VEL_MIN, OPT_VEL_MAX,
    OPT_VEL_STEP, OPT_PREROLL, OPT_PAUSE, OPT_RATE, OPT_CHANNELS, OPT_BITS,
    OPT_GAIN, OPT_TRIM, OPT_THRESH, OPT_FADE, OPT_LATENCY, OPT_LAYOUT,
    OPT_SKIP, OPT_SF2ONLY, OPT_NOSF2, OPT_SFREL, OPT_NOEXT, OPT_DRY
};

static const struct option LONGOPTS[] = {
    {"help", 0, 0, 'h'}, {"midi-port", 1, 0, 'm'},
    {"audio-device", 1, 0, 'a'}, {"length", 1, 0, 'l'},
    {"release", 1, 0, 'r'}, {"name", 1, 0, 'n'}, {"outdir", 1, 0, 'o'},
    {"list-midi", 0, 0, OPT_LIST_MIDI}, {"list-audio", 0, 0, OPT_LIST_AUDIO},
    {"channel", 1, 0, OPT_CHANNEL}, {"program", 1, 0, OPT_PROGRAM},
    {"bank", 1, 0, OPT_BANK}, {"note-min", 1, 0, OPT_NOTE_MIN},
    {"note-max", 1, 0, OPT_NOTE_MAX}, {"note-step", 1, 0, OPT_NOTE_STEP},
    {"vel-min", 1, 0, OPT_VEL_MIN}, {"vel-max", 1, 0, OPT_VEL_MAX},
    {"vel-step", 1, 0, OPT_VEL_STEP}, {"preroll", 1, 0, OPT_PREROLL},
    {"pause", 1, 0, OPT_PAUSE}, {"rate", 1, 0, OPT_RATE},
    {"channels", 1, 0, OPT_CHANNELS}, {"bits", 1, 0, OPT_BITS},
    {"gain", 1, 0, OPT_GAIN}, {"trim-silence", 0, 0, OPT_TRIM},
    {"threshold", 1, 0, OPT_THRESH}, {"fade", 1, 0, OPT_FADE},
    {"latency", 1, 0, OPT_LATENCY}, {"layout", 1, 0, OPT_LAYOUT},
    {"skip-existing", 0, 0, OPT_SKIP}, {"sf2-only", 0, 0, OPT_SF2ONLY},
    {"no-sf2", 0, 0, OPT_NOSF2}, {"sf-release", 1, 0, OPT_SFREL},
    {"no-extend", 0, 0, OPT_NOEXT}, {"dry-run", 0, 0, OPT_DRY},
    {0, 0, 0, 0}};

static double num(const char *opt, const char *s, double lo, double hi)
{
    char *end;
    double v = strtod(s, &end);
    if (end == s || *end || v < lo || v > hi) {
        fprintf(stderr, "autosampler: bad value '%s' for %s (allowed %g..%g)\n",
                s, opt, lo, hi);
        exit(2);
    }
    return v;
}

static int note_arg(const char *opt, const char *s)
{
    int n = parse_note(s);
    if (n < 0) {
        fprintf(stderr, "autosampler: bad note '%s' for %s\n", s, opt);
        exit(2);
    }
    return n;
}

int main(int argc, char **argv)
{
    cfg_t c = {
        .outdir = ".", .audio_dev = "default", .midi_port = NULL,
        .note_lo = 36, .note_hi = 84, .note_step = 3,
        .vel_lo = 100, .vel_hi = 100, .vel_step = 32,
        .channel = 0, .program = -1, .bank = -1,
        .length_ms = 2000, .release_ms = 1000, .preroll_ms = 100,
        .pause_ms = 300, .rate = 48000, .channels = 2, .bits = 24,
        .threshold_db = -70, .fade_ms = 5, .layout = LAYOUT_FLAT,
        .extend = 1, .sf_release_ms = 300,
    };
    snprintf(c.name, sizeof c.name, "Sampled");

    int opt;
    while ((opt = getopt_long(argc, argv, "hm:a:l:r:n:o:", LONGOPTS, NULL)) != -1) {
        switch (opt) {
        case 'h': usage(); return 0;
        case 'm': c.midi_port = optarg; break;
        case 'a': c.audio_dev = optarg; break;
        case 'l': c.length_ms = num("--length", optarg, 1, 600000); break;
        case 'r': c.release_ms = num("--release", optarg, 0, 600000); break;
        case 'n': snprintf(c.name, sizeof c.name, "%s", optarg); break;
        case 'o': c.outdir = optarg; break;
        case OPT_LIST_MIDI: return list_midi();
        case OPT_LIST_AUDIO: return list_audio();
        case OPT_CHANNEL: c.channel = (int)num("--channel", optarg, 1, 16) - 1; break;
        case OPT_PROGRAM: c.program = (int)num("--program", optarg, 0, 127); break;
        case OPT_BANK: c.bank = (int)num("--bank", optarg, 0, 127); break;
        case OPT_NOTE_MIN: c.note_lo = note_arg("--note-min", optarg); break;
        case OPT_NOTE_MAX: c.note_hi = note_arg("--note-max", optarg); break;
        case OPT_NOTE_STEP: c.note_step = (int)num("--note-step", optarg, 1, 127); break;
        case OPT_VEL_MIN: c.vel_lo = (int)num("--vel-min", optarg, 1, 127); break;
        case OPT_VEL_MAX: c.vel_hi = (int)num("--vel-max", optarg, 1, 127); break;
        case OPT_VEL_STEP: c.vel_step = (int)num("--vel-step", optarg, 1, 126); break;
        case OPT_PREROLL: c.preroll_ms = num("--preroll", optarg, 0, 10000); break;
        case OPT_PAUSE: c.pause_ms = num("--pause", optarg, 0, 600000); break;
        case OPT_RATE: c.rate = (unsigned)num("--rate", optarg, 8000, 192000); break;
        case OPT_CHANNELS: c.channels = (int)num("--channels", optarg, 1, 2); break;
        case OPT_BITS:
            c.bits = (int)num("--bits", optarg, 16, 24);
            if (c.bits != 16 && c.bits != 24) {
                fprintf(stderr, "autosampler: --bits must be 16 or 24\n");
                return 2;
            }
            break;
        case OPT_GAIN: c.gain_db = num("--gain", optarg, -60, 60); break;
        case OPT_TRIM: c.trim_silence = 1; break;
        case OPT_THRESH: c.threshold_db = num("--threshold", optarg, -140, 0); break;
        case OPT_FADE: c.fade_ms = num("--fade", optarg, 0, 10000); break;
        case OPT_LATENCY: c.latency_ms = num("--latency", optarg, 0, 10000); break;
        case OPT_LAYOUT:
            if (!strcmp(optarg, "flat")) c.layout = LAYOUT_FLAT;
            else if (!strcmp(optarg, "vel")) c.layout = LAYOUT_VEL;
            else if (!strcmp(optarg, "note")) c.layout = LAYOUT_NOTE;
            else {
                fprintf(stderr, "autosampler: --layout must be flat, vel or note\n");
                return 2;
            }
            break;
        case OPT_SKIP: c.skip_existing = 1; break;
        case OPT_SF2ONLY: c.sf2_only = 1; break;
        case OPT_NOSF2: c.no_sf2 = 1; break;
        case OPT_SFREL: c.sf_release_ms = num("--sf-release", optarg, 1, 20000); break;
        case OPT_NOEXT: c.extend = 0; break;
        case OPT_DRY: c.dry_run = 1; break;
        default: fprintf(stderr, "Try --help\n"); return 2;
        }
    }

    if (c.note_hi < c.note_lo || c.vel_hi < c.vel_lo) {
        fprintf(stderr, "autosampler: max must be >= min for notes and velocity\n");
        return 2;
    }
    sanitize_name(c.name);
    if (!c.name[0])
        snprintf(c.name, sizeof c.name, "Sampled");
    if (!c.sf2_only && !c.dry_run && !c.midi_port) {
        fprintf(stderr, "autosampler: --midi-port is required (see --list-midi)\n");
        return 2;
    }

    int ns;
    sample_t *plan = build_plan(&c, &ns);
    int nn = (c.note_hi - c.note_lo) / c.note_step + 1;
    int nv = (c.vel_hi - c.vel_lo) / c.vel_step + 1;
    char a[16], b[16];
    note_name(c.note_lo, 0, a, sizeof a);
    note_name(c.note_lo + (nn - 1) * c.note_step, 0, b, sizeof b);
    double per = c.preroll_ms + c.length_ms + c.release_ms + c.pause_ms;
    printf("Instrument '%s': %d notes (%s..%s, step %d) x %d velocities "
           "(%d..%d, step %d) = %d samples\n",
           c.name, nn, a, b, c.note_step, nv, c.vel_lo,
           c.vel_lo + (nv - 1) * c.vel_step, c.vel_step, ns);
    printf("Per sample: hold %.0f ms + release %.0f ms; estimated total %.1f min\n",
           c.length_ms, c.release_ms, ns * per / 60000.0);

    if (c.dry_run) {
        for (int i = 0; i < ns; i++)
            printf("  %s\n", plan[i].path);
        free(plan);
        return 0;
    }

    char sdir[PATH_MAX], sf2path[PATH_MAX];
    snprintf(sdir, sizeof sdir, "%s/%s", c.outdir, c.name);
    snprintf(sf2path, sizeof sf2path, "%s/%s.sf2", sdir, c.name);
    if (mkdir_p(sdir) < 0) {
        perror(sdir);
        return 1;
    }

    int exit_code = 0;
    if (!c.sf2_only) {
        struct sigaction sa;
        memset(&sa, 0, sizeof sa);
        sa.sa_handler = on_signal; /* no SA_RESTART: blocking reads get EINTR */
        sigaction(SIGINT, &sa, NULL);
        sigaction(SIGTERM, &sa, NULL);

        midi_t midi = {0};
        capture_t cap = {0};
        if (midi_open(&midi, c.midi_port) < 0 ||
            cap_open(&cap, c.audio_dev, c.rate, c.channels) < 0)
            return 1;

        size_t pre = ms_to_frames(c.rate, c.preroll_ms);
        size_t len = ms_to_frames(c.rate, c.length_ms);
        size_t rel = ms_to_frames(c.rate, c.release_ms);
        size_t total = pre + len + rel;
        int32_t *buf = malloc(total * c.channels * sizeof(int32_t));
        if (!buf)
            die("out of memory");

        midi_silence(&midi, c.channel);
        if (c.bank >= 0)
            midi_cc(&midi, c.channel, 0, c.bank);
        if (c.program >= 0) {
            snd_seq_event_t ev;
            snd_seq_ev_clear(&ev);
            snd_seq_ev_set_pgmchange(&ev, c.channel, c.program);
            midi_send(&midi, &ev);
        }
        msleep(300);

        for (int i = 0; i < ns && !g_stop; i++) {
            sample_t *s = &plan[i];
            char nm[16], dir[PATH_MAX];
            note_name(s->note, 0, nm, sizeof nm);

            if (c.skip_existing && file_exists(s->path)) {
                printf("[%d/%d] %-4s vel %3d  exists, skipped\n", i + 1, ns, nm,
                       s->vel);
                continue;
            }
            snprintf(dir, sizeof dir, "%s", s->path);
            *strrchr(dir, '/') = 0;
            if (mkdir_p(dir) < 0) {
                perror(dir);
                exit_code = 1;
                break;
            }

            printf("[%d/%d] %-4s vel %3d  recording... ", i + 1, ns, nm, s->vel);
            fflush(stdout);

            int r = -1;
            for (int attempt = 0; attempt < 3 && !g_stop; attempt++) {
                r = record_sample(&c, &cap, &midi, s->note, s->vel, buf, pre,
                                  len, rel);
                if (r >= 0)
                    break;
                if (r != -EINTR)
                    printf("(audio xrun, retrying) ");
            }
            if (r < 0) {
                printf("\n");
                if (r != -EINTR) {
                    fprintf(stderr, "recording failed: %s\n", snd_strerror(r));
                    exit_code = 1;
                }
                break;
            }

            int32_t *data = buf + pre * c.channels;
            size_t n = len + rel;
            int silent;
            int64_t peak = process_sample(&c, &data, &n, &silent);
            if (silent)
                printf("WARNING: silence - is the synth connected to the input? ");
            if (wav_write(s->path, data, n, c.channels, c.rate, c.bits,
                          s->note) < 0) {
                perror(s->path);
                exit_code = 1;
                break;
            }
            printf("%.2fs  peak %.1f dBFS%s\n", (double)n / c.rate, to_db(peak),
                   peak >= 2147483000LL ? "  CLIPPED" : "");
            msleep(c.pause_ms);
        }

        midi_silence(&midi, c.channel);
        free(buf);
        snd_pcm_close(cap.pcm);
        snd_seq_close(midi.seq);
        if (g_stop)
            printf("\nInterrupted. Re-run with --skip-existing to continue.\n");
    }

    for (int i = 0; i < ns; i++)
        plan[i].present = file_exists(plan[i].path);
    write_manifest(&c, plan, ns);

    if (!c.no_sf2 && !(g_stop && !c.sf2_only)) {
        if (build_sf2(&c, plan, ns, sf2path) < 0)
            exit_code = 1;
    }

    free(plan);
    return exit_code;
}
