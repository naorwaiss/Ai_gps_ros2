import cv2
import os

def main():
    # Open the camera
    #os.environ["DISPLAY"] = ":0.0"

    cap = cv2.VideoCapture('/dev/video2')

    # Check if the camera is opened successfully
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    # Set camera resolution
    width = 3840
    height = 2160
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    # Define the window size
    window_width = 1280
    window_height = 720

    # Loop to continuously capture and display frames from the camera
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        # Check if the frame is empty
        if not ret:
            print("Error: Unable to capture frame.")
            break

        # Resize the frame to fit the window
        resized_frame = cv2.resize(frame, (window_width, window_height))

        # Display the resized frame
        cv2.imshow('Camera Feed', resized_frame)

        # Check for key press
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):  # Press 'q' to quit
            break

    # Release the camera and close all OpenCV windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

