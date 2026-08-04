# pip install matplotlib

from pathlib import Path

import cv2
from matplotlib import pyplot as plt


def main():
    file_path = Path(__file__).parent
    image_path = file_path / "data" / "images.jpg"
    img = cv2.imread(str(image_path))
    if img is None:
        raise FileNotFoundError(f"이미지를 읽을 수 없습니다: {image_path}")
    cv2.imshow("robot", img)
    plt.axis("off")
    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    plt.imshow(imgRGB)
    cv2.waitKey(30)
    plt.show()


if __name__ == "__main__":
    main()
