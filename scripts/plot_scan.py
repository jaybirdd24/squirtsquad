import numpy as np
import matplotlib.pyplot as plt
import csv
import io

DATA = """heading,dist
-0.34,88.3
-0.34,39.8
-0.32,112.5
-0.33,82.7
3.36,105.6
4.8,87.7
8.06,98.8
9.6,93.4
12.88,86.4
14.52,105.2
16.33,105.5
18.11,99.4
20.0,102.6
25.71,95.3
29.63,28.6
31.46,27.7
33.29,27.2
35.03,27.0
36.8,27.3
38.6,26.4
42.1,26.1
43.8,26.3
45.61,26.3
48.86,26.4
50.49,26.5
52.17,26.5
53.7,26.1
55.15,27.0
56.95,27.1
58.41,27.0
59.75,41.1
61.4,27.8
62.96,28.4
64.51,39.5
66.2,39.6
67.88,40.0
69.36,40.3
70.91,39.8
72.38,39.7
73.88,39.4
75.53,39.5
78.82,39.4
80.54,39.9
82.25,39.8
83.96,39.8
85.77,39.5
87.69,40.4
89.48,40.0
91.11,39.1
92.8,39.3
94.61,39.2
96.66,39.8
98.69,40.2
100.56,39.9
102.39,40.0
104.09,40.1
105.56,40.3
108.87,40.5
110.49,40.7
112.17,40.5
114.1,41.9
115.84,42.6
118.94,42.4
120.51,42.9
122.17,65.5
123.64,65.6
125.26,63.9
130.22,64.3
132.06,64.5
133.94,65.3
135.76,64.1
137.48,64.6
142.69,65.4
144.51,65.5
148.06,50.5
149.84,32.1
151.62,49.9
155.16,47.6
158.69,12.8
160.2,13.1
166.88,11.2
168.9,11.1
170.61,11.0
174.95,10.9
176.57,10.8
179.76,46.1
181.28,46.2
182.95,46.2
186.54,46.9
190.09,46.5
191.54,46.6
193.29,46.7
195.08,46.4
196.78,46.3
199.84,46.8
203.44,48.5
205.32,48.2
207.2,48.5
210.4,68.1
217.61,78.3
219.3,55.0
221.21,56.6
225.29,54.2
228.77,55.3
230.42,53.6
232.08,54.4
233.77,53.6
235.63,53.0
239.04,53.4
240.7,54.3
242.45,54.6
244.07,53.7
245.67,54.1
247.27,54.7
250.93,54.4
253.98,54.6
255.43,56.0
256.98,55.3
258.47,55.4
260.17,54.7
263.42,57.4
265.05,56.5
266.71,60.5
268.64,59.4
270.43,60.3
272.07,60.9
273.81,60.5
276.91,60.7
278.53,61.8
280.22,61.3
281.9,61.9
285.29,62.3
287.09,62.9
290.97,62.3
292.68,63.0
294.5,64.7
296.09,62.6
297.58,64.5
299.32,61.1
300.91,62.8
302.66,61.3
304.5,61.4
308.15,61.8
309.89,61.4
311.35,61.8
313.1,61.4
314.89,61.9
318.58,61.6
320.32,62.8
322.08,61.7
323.74,63.0
325.38,62.0
327.1,61.9
328.82,63.8
330.51,61.9
332.49,62.5
334.56,61.8
336.4,61.2
338.01,58.9
339.65,56.1
342.62,58.2
344.03,55.6
345.67,55.4
350.65,85.6
352.26,128.4
354.08,102.1
355.9,94.3
357.78,90.3
359.76,87.7
361.7,94.6"""

reader = csv.DictReader(io.StringIO(DATA))
headings, dists = [], []
for row in reader:
    headings.append(float(row["heading"]))
    dists.append(float(row["dist"]))

angles_rad = np.deg2rad(headings)

fig, (ax_polar, ax_cart) = plt.subplots(1, 2, figsize=(14, 6),
                                         subplot_kw={"projection": None})
fig.patch.set_facecolor("#1a1a2e")

# --- Polar plot ---
ax_polar.remove()
ax_polar = fig.add_subplot(1, 2, 1, projection="polar")
ax_polar.set_facecolor("#16213e")
scatter = ax_polar.scatter(angles_rad, dists, c=dists, cmap="plasma",
                           s=18, alpha=0.85, zorder=3)
ax_polar.plot(np.append(angles_rad, angles_rad[0]),
              np.append(dists, dists[0]),
              color="cyan", linewidth=0.8, alpha=0.5, zorder=2)
ax_polar.set_theta_zero_location("N")
ax_polar.set_theta_direction(-1)
ax_polar.tick_params(colors="white")
ax_polar.set_title("360° Distance Scan (polar)", color="white", pad=15)
for spine in ax_polar.spines.values():
    spine.set_edgecolor("#444")
ax_polar.grid(color="#334", linewidth=0.5)

# --- Cartesian plot ---
ax_cart.set_facecolor("#16213e")
ax_cart.scatter(headings, dists, c=dists, cmap="plasma", s=18, alpha=0.85, zorder=3)
ax_cart.plot(headings, dists, color="cyan", linewidth=0.8, alpha=0.5, zorder=2)
ax_cart.set_xlabel("Heading (°)", color="white")
ax_cart.set_ylabel("Distance (cm)", color="white")
ax_cart.set_title("Distance vs Heading (cartesian)", color="white")
ax_cart.tick_params(colors="white")
ax_cart.set_facecolor("#16213e")
for spine in ax_cart.spines.values():
    spine.set_edgecolor("#444")
ax_cart.grid(color="#334", linewidth=0.5)
ax_cart.set_xlim(-5, 370)
ax_cart.set_xticks(range(0, 361, 45))

cb = fig.colorbar(scatter, ax=ax_cart, pad=0.02)
cb.set_label("Distance (cm)", color="white")
cb.ax.yaxis.set_tick_params(color="white")
plt.setp(cb.ax.yaxis.get_ticklabels(), color="white")

fig.suptitle("Scan360 — Robot Environment Map", color="white", fontsize=14, y=1.01)
plt.tight_layout()
plt.savefig("scan_plot.png", dpi=150, bbox_inches="tight",
            facecolor=fig.get_facecolor())
print("Saved scan_plot.png")
plt.show()
