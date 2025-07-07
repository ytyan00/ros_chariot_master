import numpy as np
import cv2

def generate_hamming_marker(marker_id):
    if not (0 <= marker_id <= 1023):
        raise ValueError("Marker ID must be between 0 and 1023")

    # Convert marker_id to 10-bit binary string
    bin_str = format(marker_id, '010b')  # 10-bit binary
    rows = [bin_str[i:i+2] for i in range(0, 10, 2)]  # Split into 5 rows

    # Define the 5x5 grid (inner marker)
    marker_inner = np.zeros((5, 5), dtype=np.uint8)

    # Encode each row following the pattern
    for i, row_bits in enumerate(rows):
        row_int = int(row_bits, 2)  # Convert "00", "01", "10", "11" to integers

        # Encoding rule:
        # First bit: Inverse of Hamming parity (simplified as 1 for all rows)
        marker_inner[i, 0] = 1

        # Next four bits based on encoding rules
        if row_int == 0:  # "00"
            marker_inner[i, 1:] = [0, 0, 0, 0]
        elif row_int == 1:  # "01"
            marker_inner[i, 1:] = [0, 1, 1, 1]
        elif row_int == 2:  # "10"
            marker_inner[i, 1:] = [1, 0, 0, 1]
        elif row_int == 3:  # "11"
            marker_inner[i, 1:] = [1, 1, 1, 0]

    # Create a 7x7 marker and pad with black border
    marker_padded = np.zeros((7, 7), dtype=np.uint8)
    marker_padded[1:6, 1:6] = marker_inner  # Place 5x5 marker inside 7x7 frame

    return marker_padded

def display_marker(marker):
    # Scale the marker for visualization
    marker_resized = cv2.resize(marker * 255, (350, 350), interpolation=cv2.INTER_NEAREST)
    cv2.imshow("Hamming Code Fiducial Marker", marker_resized)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Example: Generate marker for ID = 110 with padding
marker_id = 110
marker = generate_hamming_marker(marker_id)
display_marker(marker)
