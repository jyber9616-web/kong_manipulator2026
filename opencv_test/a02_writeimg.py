from pathlib import Path

import cv2


def main():
    file_path = Path(__file__).parent
    image_path = file_path / "data" / "images.jpg"
    img = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"이미지를 읽을 수 없습니다: {image_path}")

    print(type(img), img.shape, img.dtype)
    cv2.imshow("images", img)

    # 창을 닫거나 아무 키나 누르면 종료한다.
    while cv2.getWindowProperty("images", cv2.WND_PROP_VISIBLE) >= 1:
        if cv2.waitKey(50) != -1:
            break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
