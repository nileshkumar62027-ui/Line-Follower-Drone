How the algorithm works

Idea: find the start end of the line first, fly there, follow the whole line to the end, then land on the circle. Finding the start first matters because every completed section earns a point.

Think of a person following a painted line with a small torch, dropping breadcrumbs behind them so they always know which way is "forward".

Building blocks
Seeing red. Each pixel is tested with colour ratios (red much larger than green and blue), which is more robust to lighting than a fixed threshold. Red pixels are grouped into separate blobs: the line, the circle, and any other lines are different blobs.
Pixels to metres (self-calibration). After takeoff the drone makes two tiny test hops (8 cm north, then 8 cm east) and measures how the camera image shifts. This gives the camera-to-position mapping (mapIdx, 0 to 7) and the metres-per-pixel scale (fovw).
Breadcrumb trail. The drone remembers every place it has been. It aims at the red point of its own line that is farthest from all breadcrumbs. That point is always forward, even at a sharp corner. Only points within a few centimetres of the best one are averaged, so it aims at one arm and not between two.
Start end or circle end. At a line end the code counts how many times the line crosses an imaginary ring: 1 crossing means a line end, 2 or more means the middle or a corner. An end with no circle next to it is the start.
Circle detection. The circle is an isolated, round, compact blob of the right size relative to the line width. It is only accepted just past the end of the line, so a corner is never mistaken for it.
Speed control. On straight lines the drone aims further ahead (fast); in sharp turns it aims only a few centimetres ahead (slow and accurate).
Remembered landing spot. Once the whole circle is seen, its centre is converted to a floor position and remembered. During descent the drone keeps steering to that point, even when the circle fills the image.
States (dbg(1) on the Scope)
State	What the drone is doing
0	Takeoff to hover height; measure the line width
7, 8	Calibration hops (north, then east)
9	Return to the takeoff point and check whether it is already at the start end
14	Survey: climb high and look for a line end with no circle (the start)
15	Fly to that start end at survey height and re-check from above
17	Descend straight onto the start
16	Survey failed: come back down
10	Spiral search (no red visible at all)
11	Lock onto the line and work out where on the line the drone is
1	Follow the line with the trail and corner-aware speed
3	Line ended: creep forward a little and look for the circle
12	Turn around at a plain end (it must be the start)
13	Fallback: slow circle hunt
4	Centre over the circle
5	Descend softly
6	Landed

dbg(2) is the route: 0 start not known, 1 reached the circle end first and is heading to the start, 2 start known and doing the full run. dbg(4) is the calibration mapping and dbg(5) is the estimated footprint fovw.

Soft and accurate landing
Descent speed is 0.40 m/s above 0.4 m and 0.12 m/s below it, with touchdown at 0.08 m.
The drone only keeps descending while it is close to the remembered circle centre.
