# from pathlib import Path

import color
import cv2
import numpy as np


def main():
    img = np.zeros((300, 700, 3), dtype=np.uint8)
    pt1, pt2 = (180, 150), (550, 150)
    size = (120, 60)

    cv2.circle(img, pt1, 1, color.RED, 2)
    cv2.circle(img, pt2, 1, color.RED, 2)

    cv2.ellipse(img, pt1, size, 0, 0, 360, color.BLUE, 1)  # type: ignore
    cv2.ellipse(img, pt2, size, 90, 0, 360, color.BLUE, 1)  # type: ignore
    cv2.ellipse(img, pt1, size, 0, 30, 270, color.ORANGE, 4)  # type: ignore
    cv2.ellipse(img, pt2, size, 90, -45, 90, color.ORANGE, 4)  # type: ignore
    cv2.imshow("canvas", img)
    cv2.waitKey()  # 블럭 함수
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
