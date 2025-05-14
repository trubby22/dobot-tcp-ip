import cv2
import numpy as np
from scipy.optimize import linear_sum_assignment

def detect_markers(image):
    """Detect markers using blob detection and return their centroids."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    centroids = []
    for cnt in contours:
        M = cv2.moments(cnt)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            centroids.append((cx, cy))
    return np.array(centroids)

# Load reference image and detect markers
ref_image = cv2.imread('reference.png')
ref_centroids = detect_markers(ref_image)
assert len(ref_centroids) == 20, "Reference image must have exactly 20 markers."

# Sort markers by y then x coordinates (adjust based on your sensor's layout)
ref_centroids = sorted(ref_centroids, key=lambda pt: (pt[1], pt[0]))
ref_centroids = np.array(ref_centroids)

########################################################

cap = cv2.VideoCapture('input_video.mp4')

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    current_centroids = detect_markers(frame)
    if len(current_centroids) != 20:
        print(f"Skipping frame with {len(current_centroids)} markers detected.")
        continue

    # Match markers using the Hungarian algorithm to minimize total distance
    cost_matrix = np.linalg.norm(ref_centroids[:, np.newaxis] - current_centroids, axis=2)
    row_ind, col_ind = linear_sum_assignment(cost_matrix)
    matched_current = current_centroids[col_ind]

    # Compute displacement vectors
    displacement_vectors = matched_current - ref_centroids

    # Visualize displacements (optional)
    vis = frame.copy()
    for i, (ref_pt, curr_pt) in enumerate(zip(ref_centroids, matched_current)):
        cv2.line(vis, tuple(ref_pt.astype(int)), tuple(curr_pt.astype(int)), (0, 255, 0), 2)
        cv2.circle(vis, tuple(curr_pt.astype(int)), 3, (0, 0, 255), -1)
    cv2.imshow('Displacements', vis)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()