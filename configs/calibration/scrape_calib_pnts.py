import json

# edit this list to your actual filenames, in the order you want the points
FILENAMES = [
    "configs/calibration/c1.json",
    "configs/calibration/c2.json",
    "configs/calibration/c3.json",
    "configs/calibration/c4.json",
    "configs/calibration/c5.json",
    "configs/calibration/c6.json",
    "configs/calibration/c7.json",
    "configs/calibration/c8.json",
]


def scrape_calibration_points(filenames):
    points = []
    for fname in filenames:
        with open(fname) as f:
            data = json.load(f)
        x, y = data["tcp_pose"][0], data["tcp_pose"][1]
        points.append((x, y))
        print(f"{fname}: x={x:.6f}, y={y:.6f}")
    return points


if __name__ == "__main__":
    calibration_points = scrape_calibration_points(FILENAMES)

    print("\nCALIBRATION_ROBOT_POINTS = [")
    for x, y in calibration_points:
        print(f"    ({x:.6f}, {y:.6f}),")
    print("]")

'''

 python scrape_calib_pnts.py 
c1.json: x=-0.324515, y=-0.535946
c2.json: x=-0.324433, y=-0.665751
c3.json: x=-0.325629, y=-0.860529
c4.json: x=-0.453511, y=-0.536058
c5.json: x=-0.455554, y=-0.731123
c6.json: x=-0.455542, y=-0.861135
c7.json: x=-0.583241, y=-0.599751
c8.json: x=-0.585706, y=-0.860013

CALIBRATION_ROBOT_POINTS = [
    (-0.324515, -0.535946),
    (-0.324433, -0.665751),
    (-0.325629, -0.860529),
    (-0.453511, -0.536058),
    (-0.455554, -0.731123),
    (-0.455542, -0.861135),
    (-0.583241, -0.599751),
    (-0.585706, -0.860013),
]

'''