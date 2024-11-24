import csv
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d

def processLidarData(input_file):
    print(f"Processing {input_file}.")
    global lidarX, lidarY
    with open(input_file, 'r') as infile:
        reader = csv.DictReader(infile)
   
        lidarData = []
        for row in reader:
            time = int(row['time'])
            point = float(row['points'])
            points2 = row[None]
            
            x = [point]
            y = []
            is_x = False
            for p in points2:
                if (len(p) > 0):
                    if (float(p) != 0.0):
                        if is_x:
                            x.append(float(p))
                        else:
                            y.append(float(p))
                is_x = not is_x

            lidarData.append((time, x, y))
        return lidarData
    
def processRoverData(lidar_file, state_file):
    print(f"Processing {state_file}.")
    # Read lidar data
    with open(lidar_file, 'r') as lf:
        lidar_reader = csv.reader(lf)
        lidar_header = next(lidar_reader)
        lidar_data = [(float(row[0]), row[1:]) for row in lidar_reader]
    
    # Read state data
    with open(state_file, 'r') as sf:
        state_reader = csv.reader(sf)
        state_header = next(state_reader)
        state_data = [list(map(float, row)) for row in state_reader]

    # Extract timestamps and state columns
    index = 0
    for i in range(1,len(lidar_data)):
        if (lidar_data[i][0] - lidar_data[index][0] < 10):
            print(i)
        else:
            index = i

    
    lidar_timestamps = [t for t, _ in lidar_data]
    state_timestamps = [row[0] for row in state_data]
    state_values = np.array([row[1:] for row in state_data])

    # Extract timestamps and state columns
    lidar_timestamps = [t for t, _ in lidar_data]
    state_timestamps = [row[0] for row in state_data]
    state_values = np.array([row[1:] for row in state_data])

    # Interpolate state data for lidar timestamps
    interpolated_state = np.array([
        interp1d(state_timestamps, state_values[:, i], fill_value="extrapolate")(lidar_timestamps)
        for i in range(state_values.shape[1])
    ]).T

    return interpolated_state

def mergeAndWrite(filename, lidarData, roverData):
    print(f"Merging lidar and rover data to {filename}.")
    with open(filename, 'w') as file:
        for i in range(1, len(roverData)):
            t = lidarData[i][0] - lidarData[0][0]
            x = lidarData[i][1]
            y = lidarData[i][2]
            yaw = np.deg2rad(roverData[i][2])
            posx = roverData[i][9]
            posy = roverData[i][10]

            if (len(x) > len(y)): # Sjekke at det er like mange x og y verdier
                x = x[:len(y)]
            newX = []
            newY = []

            file.write(f"{t};{posx};{posy};{yaw};")
            for i in range(len(x)):
                if (abs(x[i]) > 100 or abs(y[i]) > 100): # Fjerne infinity points som kommer av dårlig data
                    continue
                
                if (np.isnan(x[i]) or np.isnan(y[i])):
                    continue
                file.write(f"{x[i]},{y[i]}:")
                _x = x[i]*np.cos(yaw) - y[i]*np.sin(yaw)+posy
                _y = x[i]*np.sin(yaw) + y[i]*np.cos(yaw)+posx

                newX.append(_x) # Rotere med yaw og plusse på posisjon
                newY.append(_y) # Posisjon er ikke linet opp med lidar sin x og y tror jeg.
            file.write("\n")


if __name__ == "__main__":
    which = "SofaRom"
    lidarData = processLidarData(f"LidarData{which}.csv")
    roverData = processRoverData(f"LidarData{which}.csv", f"RoverData{which}.csv")
    mergeAndWrite(f"ProcessedData{which}.txt", lidarData, roverData)