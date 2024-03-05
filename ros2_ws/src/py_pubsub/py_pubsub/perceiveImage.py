import cv2
import matplotlib.pyplot as plt
import time


def main():
    # Open a connection to the webcam (you can change the argument to 1 if you have multiple cameras)
    cap = cv2.VideoCapture(0)

    # Check if the camera opened successfully
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    while True:
        ret, frame = cap.read()

        # Read a frame from the webcam
        # Check if the frame was read successfully
        if not ret:
            print("Error: Could not read frame.")
        
        # Desired dimension
        height, width = 300, 300

        # Define new size of your frame
        frame = resize_frame(frame, height, width)

        # Convert to gray
        frame_gray = convert_to_gray(frame)

        # Let’s load the pre-trained Haar Cascade classifier that is built into OpenCV:
        face_classifier = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")

        # DetectMultiScale
        face = face_classifier.detectMultiScale(frame_gray, scaleFactor=1.1, minNeighbors=5, minSize=(40, 40))

        for (x, y, w, h) in face:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 4)    


        fps_start_time = time.time()
        fps = calculate_fps(fps_start_time)
        # Display fps on the frame
        fps_str = str(fps) 

        cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)        # Display the frame in a window
        cv2.imshow('Face Detection', frame)

        # Break the loop when the user presses the 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the webcam and close the window
    cap.release()
    cv2.destroyAllWindows()


def resize_frame(frame, width, height):
    # Resize the frame to the specified width and height
    return cv2.resize(frame, (width, height))


def convert_to_gray(image):
    # Convert to gray
    frame_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return frame_gray

def  calculate_fps(fps_start_time): 
    fps_counter = 0
    fps = 0
    # Calculate fps
    fps_counter += 1
    fps = fps_counter / (time.time() - fps_start_time)
    return fps

if __name__ == "__main__":
    main()
