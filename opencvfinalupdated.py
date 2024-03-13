#importing libraries
import cv2
import numpy as np

# Load the volleyball match video
video = cv2.VideoCapture("/Users/rohanshenoy/Downloads/volleyball_match.mp4")

# Load the volleyball template image
template = cv2.imread("/Users/rohanshenoy/Downloads/WhatsApp Image 2024-03-13 at 15.35.44.jpeg", cv2.IMREAD_GRAYSCALE)

# Get the width and height of the template
w, h = template.shape[::-1]

# Load the pre-trained Haar cascade for full-body detection
fullbody_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_fullbody.xml')

# Define the codec and create VideoWriter object
#we create a videowriter obj to save the processed video
fourcc = cv2.VideoWriter_fourcc(*'XVID')
output_video = cv2.VideoWriter('output.avi', fourcc, 30.0, (int(video.get(3)), int(video.get(4))))

# Initialize variables for trajectory tracking
prev_ball_center = None

# Initialize player counting variables
num_team1_players = 0
num_team2_players = 0

#looping thru each frame of the video
while video.isOpened():
    ret, frame = video.read()
    if not ret:
        break

    # Convert frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Perform template matching for ball detection
    res = cv2.matchTemplate(gray, template, cv2.TM_CCOEFF_NORMED)

    # Define a threshold for the match score to seperate object from background
    threshold = 0.77

    
    # Get the location of the match where it is above our threshold values
    loc = np.where(res >= threshold)

    # Draw a rectangle around the matched region
    for pt in zip(*loc[::-1]):
        cv2.rectangle(frame, pt, (pt[0] + w, pt[1] + h), (0, 255, 0), 2)

        # Update the trajectory of the ball
        ball_center = (pt[0] + w // 2, pt[1] + h // 2)
        if prev_ball_center is not None:
            cv2.line(frame, prev_ball_center, ball_center, (0, 0, 255), 2)
        prev_ball_center = ball_center

    # Detect players using Haar cascade for upper body
    bodies = fullbody_cascade.detectMultiScale(gray, scaleFactor=1.3, minNeighbors=6, minSize=(50, 50))
    
    #if the player is detected we check whether he is in the upper or lower half of the screen and classify him in team
    # Loop over the detected players
    for (x, y, w, h) in bodies:
        # Check if player is in the lower half or upper half of the frame
        
        if y + h < 2*(frame.shape[0]) // 3: 
            # Lower half (Team 1)
            if num_team1_players < 6:
                num_team1_players += 1
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)  
                
        else: 
            # Upper half (Team 2)
            if num_team2_players < 6:
                num_team2_players += 1
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)  

    # Draw text showing the number of players on each team
    cv2.putText(frame, f"Team 1: {num_team1_players} players", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
    cv2.putText(frame, f"Team 2: {num_team2_players} players", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    # Write the frame to the output video
    output_video.write(frame)

    # Display the frame with volleyball and player detections
    cv2.imshow('Volleyball Tracking', frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(25) & 0xFF == ord('q'):
        break

# Release the video objects
video.release()
output_video.release()

# Close all OpenCV windows
cv2.destroyAllWindows()