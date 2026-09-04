# media/ — status

The README references these. Current state:

| file | status |
|---|---|
| `banner.png` / `banner.svg` | ✅ provided (banner + editable source) |
| `demo_overview.png` | ✅ provided — full frame from your VID-…WA0016 (terminal choices + sim + RViz map) |
| `harness_choice.png` | ✅ provided — terminal close-up of the two-candidate prompt + choices |
| `map.png` | ⚠️ stopgap (tilted SLAM view). Recapture a clean **top-down** for a stronger image — see below |
| video | ⬜ drag `VID-…WA0016.mp4` into the README on GitHub (see below) |

## Better map.png (2 min, optional)
Run Phase 2, and in RViz orbit the camera to look **straight down** at the map
(or set Views → Orbit Pitch to ~1.57), enable **Map** + **RobotModel**, frame the
whole room, and screenshot the render area only. Save as `media/map.png`.

## Adding the video (inline player)
1. On GitHub, open `README.md` → pencil (**Edit**).
2. Put the cursor on the blank line under `### 🎥 Video Demo`.
3. **Drag `VID-…WA0016.mp4` into the editor box.** It's ~5 MB (under GitHub's 10 MB
   limit), so it uploads and inserts a `user-attachments` link that renders as an
   inline video player.
4. **Commit changes.**

If a file is ever over 10 MB: shrink with
`ffmpeg -i in.mp4 -vcodec libx264 -crf 28 -an out.mp4`, or upload to YouTube
(Unlisted) and use a clickable thumbnail:
`[![demo](media/video_thumb.png)](https://youtu.be/ID)`.
