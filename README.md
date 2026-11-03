# autosampler

Command-line MIDI autosampler for Linux (C, ALSA). It plays notes to a synth
over MIDI, records the audio coming back, saves one WAV per note/velocity and
builds an SF2 SoundFont and an SFZ text file that map the whole set.

## Build

    sudo apt install build-essential libasound2-dev     # Debian/Ubuntu
    make

## Quick start

    ./autosampler --list-midi            # find the synth's MIDI port, e.g. 128:0
    ./autosampler --list-audio           # find the capture device, e.g. hw:1,0

    ./autosampler -m 128:0 -a hw:1,0 -n MyPiano \
        --note-min C1 --note-max C7 --note-step 3 \
        --vel-min 32 --vel-max 127 --vel-step 32 \
        -l 4000 -r 1500 --trim-silence

Use `--dry-run` first to see the file list and estimated time.

## What gets recorded

* Notes: `--note-min` .. `--note-max` every `--note-step` semitones
  (numbers or names; middle C = C4 = 60).
* Velocities: `--vel-min` .. `--vel-max` every `--vel-step`. Only values
  reached by the step are sampled (32..127 step 32 -> 32, 64, 96).
* Each sample = note held for `--length` ms, then `--release` ms of tail.
  Timing comes from the audio clock, so it does not drift.
* `--preroll` ms of capture before note-on, `--pause` ms between samples.
* `--latency MS` removes round-trip latency from the start; `--trim-silence`
  instead finds the onset automatically (and trims the silent tail).
  `--fade` applies a short fade-out so tails never click.

## Output layout

    <outdir>/<name>/<name>_<note#>_<NoteName>_v<vel>.wav     (--layout flat)
    <outdir>/<name>/v<vel>/...                               (--layout vel)
    <outdir>/<name>/<note#>_<NoteName>/...                   (--layout note)
    <outdir>/<name>/<name>.sf2
    <outdir>/<name>/<name>.sfz
    <outdir>/<name>/<name>_manifest.csv

File names sort by note, then velocity (`MyPiano_060_C4_v096.wav`;
sharps are written `Cs`, e.g. `MyPiano_061_Cs4_v096.wav`). WAVs are 16 or 24
bit (`--bits`), mono or stereo, and carry the root key in a `smpl` chunk.

## SoundFont

* One instrument/preset (bank 0, preset 0) containing one zone per sample
  (two for stereo, panned L/R and linked).
* Key zones are split halfway between neighbouring sampled notes; the outer
  zones stretch to 0 and 127 (`--no-extend` to disable).
* Velocity layers are split halfway between sampled velocities, the lowest
  layer starting at 1 and the highest ending at 127.
* Root key = the sampled note. Samples are one-shots (no loops); the release
  tail is part of the audio, and `--sf-release` sets the envelope release used
  when a key is let go (default 300 ms).
* SF2 audio is always 16-bit.

## SFZ

`<name>.sfz` is a plain-text instrument that references the WAV files
directly (paths relative to the .sfz, so keep the folder together). It uses
the same key/velocity zones as the SF2: one `<group>` per sampled note
(`lokey`, `hikey`, `pitch_keycenter`) containing one `<region>` per velocity
layer (`lovel`, `hivel`, `sample`). Global settings: `loop_mode=no_loop` and
`ampeg_release` from `--sf-release`. Use `--no-sfz` to skip it.

## Resuming / rebuilding

* Ctrl-C stops cleanly (note-off + all-sound-off are sent).
* `--skip-existing` continues an interrupted session or fills gaps.
* `--sf2-only` rebuilds the SoundFont and SFZ (with the same note/velocity/name/layout
  options) from WAVs already on disk, no hardware needed.

## Notes

* Connect the synth's audio output to the capture device's input. Keep the
  levels sane; the tool prints each sample's peak and flags clipping, and
  `--gain` can adjust the result.
* If `hw:` rejects the format or rate, use the `plughw:` variant of the device.
* Software synths: start e.g. `fluidsynth -a jack` / Pianoteq / a JACK-routed
  synth, route its output to the capture device (or a loopback card such as
  `snd-aloop`), and point `--midi-port` at its ALSA sequencer port.
