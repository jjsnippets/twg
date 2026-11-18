# OBSBOT Tiny SE V4L2 movement and zoom demo

`demo_ptz.c` controls an OBSBOT Tiny SE connected to the Raspberry Pi.
It traces one clockwise pan–tilt square, requests center twice, then
steps through five absolute zoom commands without changing pan or tilt.

The demo uses Linux V4L2 directly. It does not capture video, start or
stop the streaming processes, or require OpenCV, the OBSBOT SDK, or
OBSBOT Center on the Pi. Start the existing 720p30 WebRTC pipeline
manually and use the Chrome preview to verify physical movement.

## Build

Run from the `twg` repository root on the Pi. A C compiler and the Linux
userspace headers providing `<linux/videodev2.h>` are required.

```bash
mkdir -p ptz/build
cc -std=c11 -O2 -Wall -Wextra -Wpedantic \
  ptz/demo_ptz.c -o ptz/build/demo_ptz
```

No Makefile or extra link library is required. The repository's `build/`
ignore rule covers `ptz/build/`.

## Prepare the camera

1. Place the camera on a stable surface with clearance for the gimbal
   and USB cable. Aim it at a stationary scene with visible landmarks.
2. Keep AI tracking, gesture control, and auto zoom off.
3. Confirm the Tiny SE's image-capture node. Linux device numbering can
   change after reconnecting the camera.

```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-ctrls-menus
```

The program leaves the starting zoom untouched during the square. For
consistent framing between runs, set the wide position beforehand:

```bash
v4l2-ctl -d /dev/video0 --set-ctrl=zoom_absolute=0
```

Use the verified capture node in that command, the FFmpeg command, and
the demo invocation if it differs from `/dev/video0`.

## Start the video stream manually

### Pi terminal 1: MediaMTX

Use the existing MediaMTX v1.21.4 installation and configuration. Do
not start a second server if one is already running.

```bash
cd "$HOME"
./mediamtx/mediamtx ./mediamtx/mediamtx.yml
```

### Pi terminal 2: FFmpeg publisher

```bash
ffmpeg -hide_banner \
  -f v4l2 \
  -input_format mjpeg \
  -video_size 1280x720 \
  -framerate 30 \
  -i /dev/video0 \
  -map 0:v:0 \
  -an \
  -c:v libx264 \
  -preset ultrafast \
  -tune zerolatency \
  -pix_fmt yuv420p \
  -profile:v baseline \
  -bf 0 \
  -g 30 \
  -b:v 2500k \
  -f rtsp \
  -rtsp_transport tcp \
  rtsp://127.0.0.1:8554/cam
```

Wait for MediaMTX to report that the `cam` stream is available.

### Laptop: Chrome preview

Open the stock MediaMTX player in Chrome on the wired bench network:

```text
http://192.168.50.1:8889/cam
```

Confirm that the video is playing. The RTSP/TCP connection is the local
FFmpeg-to-MediaMTX publishing leg; Chrome receives the video via WebRTC.
The custom WHEP player is not needed.

## Run the demo

In a third Pi terminal, from the repository root:

```bash
./ptz/build/demo_ptz
```

The defaults are `/dev/video0` and a 2000 ms dwell after each accepted
write. To pass a device and use a longer dwell:

```bash
./ptz/build/demo_ptz /dev/video0 3000
```

The dwell applies to the square, both center writes, and every zoom step.
Use a dwell long enough to observe the movement and framing change. It is
a fixed pause, not feedback that the gimbal or zoom has settled.

## Expected sequence

### Pan and tilt

| Step | Requested pointing direction | Pan (arcsec) | Tilt (arcsec) |
|---|---|---:|---:|
| 1 | Upper left, start | -54000 | +54000 |
| 2 | Upper right | +54000 | +54000 |
| 3 | Lower right | +54000 | -54000 |
| 4 | Lower left | -54000 | -54000 |
| 5 | Upper left, close the square | -54000 | +54000 |
| 6 | Center, first write | 0 | 0 |
| 7 | Center, second write | 0 | 0 |

The corner coordinates are ±15° on each axis relative to the commanded
center. Judge clockwise by where the camera points, not by the opposite
shift of the background in the preview.

Each row sends a paired V4L2 request. Two separate center writes are sent
even if the first one visibly centers the camera. The first write has
worked in a demo run, but manual testing showed that it was not always
sufficient. The program does not use readback to decide whether to skip
the second write.

### Zoom

After the dwell following the second center write, the demo sends:

```text
zoom_absolute: 0 → 3 → 5 → 12 → 0
```

The first `0` establishes the wide view for comparison. `12` is the
advertised maximum absolute setting on the tested unit; the values are
device control units, not magnification ratios. The final `0` restores
the wide setting. The demo does not use `zoom_continuous` or issue any
pan–tilt writes during this phase.

## Verify and stop

- Confirm that the camera visits all four corners in order and returns
  to the upper-left view before centering.
- Confirm that the camera is physically centered before the first zoom
  step and that its pointing direction does not change during zoom.
- Check for visible framing changes at `3`, `5`, and `12`, and confirm
  that the last `0` restores the centered, wide view.
- Verify that the viewer stays connected. Note any freezes, corruption,
  camera disconnects, or incomplete moves. Repeat the sequence and record
  the dwell used if checking repeatability.
- Check Pi power and throttling status before and after the demo with
  `vcgencmd get_throttled`.

`command accepted` means the V4L2 write returned successfully. It does
not certify that the camera physically arrived or that the zoom settled.
Exit status zero means all commands were sent successfully, not that the
visual checks passed.

A control error or Ctrl+C stops the sequence without sending cleanup
movement or zoom commands. It does not cancel a movement that the camera
already accepted. Check the actual pointing direction and zoom before
starting again.

To shut down streaming, close the viewer, stop FFmpeg, then stop
MediaMTX. The demo leaves both streaming processes running.
