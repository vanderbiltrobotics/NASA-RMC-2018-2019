

def filter_disparity_images(last_frames):

    # For each pixel in the images, compare the value across each of the images
    

    # If the values vary too much, sets the pixel value to 0

    # Otherwise, keep the value the same

    # return filtered image



# Write a ros node that listens to the topic where disparity images are being published

# Keep a list of the last 'n' frames

# Loop continuously

# Each time through the loop, get the latest frame, add it to the list of frames

# If list is full, remove the oldest frame

# Call the filtering function

# Publish the filtered image