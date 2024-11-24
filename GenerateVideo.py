import cv2
import numpy as np
from ReadWriteDataset import readDataset
from MergeAndFormatRawData import *
import os
def generateMappingVideo(output_file, frames, fps=10, margin=0):
    print(f"Generating mapping video to {output_file}.")
    # Find overall data bounds
    all_x = np.concatenate([lidarX for t,x,y,heading, lidarX, _ in frames])
    all_y = np.concatenate([lidarY for t,x,y,heading, _, lidarY in frames])

    min_x, max_x = all_x.min() - margin, all_x.max() + margin
    min_y, max_y = all_y.min() - margin, all_y.max() + margin

    # Determine frame size dynamically
    width = 800  # Set width of the video
    height = int((max_y - min_y) / (max_x - min_x) * width)

    # Initialize video writer
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec for MP4 video
    video_writer = cv2.VideoWriter(output_file, fourcc, fps, (width, height))

    # Scale factors to map lidar data to frame
    scale_x = width / (max_x - min_x)
    scale_y = height / (max_y - min_y)

    # Prepare each frame
    i = 0
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    for t,_x,_y,_heading,lidarX, lidarY in frames:
        # Create a blank black image

        # Convert lidar points to pixel coordinates
        lidar_points = np.array([lidarX, lidarY]).T

        lidar_points[:, 0] = ((lidar_points[:, 0] - min_x) * scale_x).astype(int)
        lidar_points[:, 1] = height - ((lidar_points[:, 1] - min_y) * scale_y).astype(int)

        _x = int(((_x-min_x) *scale_x)) # Rover pos x
        _y = int(height - ((_y-min_y) *scale_y)) # Rover pos y

        cv2.circle(frame, (_x, _y), 1, (0, 0, 255), -1)
        # Draw lidar points on the frame
        for point in lidar_points:
            x = int(point[0])
            y = int(point[1])
            if 0 <= x < width and 0 <= y < height:
                cv2.circle(frame, (x, y), 1, (0, 255, 0), -1)  # Green points

        # Write frame to video

        _frame = frame.copy()
        
        length = 20
        cv2.line(_frame, (_x, _y), (int(_x + np.cos(_heading) * length), int(_y + np.sin(-_heading) * length)), (255, 0, 0), 2)
        video_writer.write(_frame)
        i += 1
        if (i%237 == 0):
            print(f"{round(i/len(frames) * 100,2)}% complete.")
    print(f"100.00% complete.")
    # Release video writer
    video_writer.release()
    
def folderContainsFile(folder_path, filename):
    return filename in os.listdir(folder_path)

# Process raw data
which = "SofaRom"
if not folderContainsFile("data", f"ProcessedData{which}.txt"):
    lidarData = processLidarData(f"data/LidarData{which}.csv")
    roverData = processRoverData(f"data/LidarData{which}.csv", f"data/RoverData{which}.csv")
    mergeAndWrite(f"data/ProcessedData{which}.txt", lidarData, roverData)

which = "Gang"
if not folderContainsFile("data", f"ProcessedData{which}.txt"):
    lidarData = processLidarData(f"data/LidarData{which}.csv")
    roverData = processRoverData(f"data/LidarData{which}.csv", f"data/RoverData{which}.csv")
    mergeAndWrite(f"data/ProcessedData{which}.txt", lidarData, roverData)

frames = readDataset("data/ProcessedDataSofaRom.txt", 0, 0, np.deg2rad(177))
frames = frames + readDataset("data/ProcessedDataGang.txt", -5.2, -3.2, 2.4)
generateMappingVideo("videos/CombinedScan1.mp4", frames, fps=120, margin=0)