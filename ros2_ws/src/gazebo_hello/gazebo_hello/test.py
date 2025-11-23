#!/usr/bin/env python3
import cv2

def main():
    # Change this if your image has a different name
    image_path = "face2.png"

    # Load image
    img = cv2.imread(image_path)
    print(f"Detecting face: {image_path}")
    if img is None:
        print(f"❌ Could not load image: {image_path}")
        return
    
    # Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Load Haar Cascade
    face_cascade = cv2.CascadeClassifier(
        cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
    )

    # Detect faces
    faces = face_cascade.detectMultiScale(
        gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30)
    )

    # Print results
    if len(faces) == 0:
        print("❌ No faces detected.")
    else:
        print(f"✅ {len(faces)} face(s) detected.")

        # Draw boxes for visualization
        for (x, y, w, h) in faces:
            cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)

        # Show the result
        cv2.imshow("Detection Result", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
