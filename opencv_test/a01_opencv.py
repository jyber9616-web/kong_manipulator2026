# python a01_opencv.py
import cv2
import numpy as np
from pathlib import Path

def main():
    file_path = Path(__file__).parent
    print("안녕하세요.")
    print(cv2.__version__)
    #black_img = np.array((300, 300, 1), dtype=np.uint8)
    #cv2.imshow("black", black_img)
    img = cv2.imread(str(file_path/"data/images.jpg"))
    cv2.imshow("images", img)
    cv2.waitKey()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()