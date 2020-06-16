# Mcity NAVYA Driverless Shuttle Trip Analysis
# By Jack Behrend

import json
import os
import math
import csv
import matplotlib.pyplot as plt

class TripReader:
    # Constructor: takes the trip's JSON path/filename,
    # sets instance variables for this trip
    def __init__(self, filename):
        self.current_pedestrian_count = -1 # Init -1 b/c ped. always nonnegative
        self.num_events = 0
        self.filename = filename
        self.start_time = 0
        self.init_data = 0
        self.current_time = 0
        with open(self.filename) as f:
            self.data = json.load(f)

    # Keeps track of abnormal pedestrian counts, negative timestamps,
    # and abnormal time jumps
    def identify_abnormal_cases(self):
        abn_ped_count = 0
        gnss_neg_time = 0
        trip_obj_neg_time = 0
        gnss_abn_jump = 0
        gnss_jump_list = []
        trip_obj_abn_jump = 0

        # Constants
        GNSS_DIFF_THRESH = 500
        TRIP_OBJ_DIFF_THRESH = 100
        PED_THRESH = 10

        initial_data = True
        last_time = 0
        for gnss_obj in self.data['trip_gnss']:
            if gnss_obj['milliseconds'] < 0:
                gnss_neg_time += 1

            if initial_data:
                last_time = gnss_obj['milliseconds']
                initial_data = False
            else:
                time_diff = gnss_obj['milliseconds'] - last_time
                if time_diff > GNSS_DIFF_THRESH:
                    gnss_abn_jump += 1

                    # Start time, end time, difference
                    gap_info = [last_time, gnss_obj['milliseconds'], time_diff]
                    gnss_jump_list.append(gap_info)

                last_time = gnss_obj['milliseconds']

        initial_data = True
        last_time = 0
        for trip_obj in self.data['trip_objects_forward']:
            if trip_obj['milliseconds'] < 0:
                trip_obj_neg_time += 1

            if trip_obj['pedestrian_count'] > PED_THRESH:
                abn_ped_count += 1

            if initial_data:
                last_time = trip_obj['milliseconds']
                initial_data = False
            else:
                time_diff = trip_obj['milliseconds'] - last_time
                if time_diff != TRIP_OBJ_DIFF_THRESH:
                    trip_obj_abn_jump += 1
                last_time = trip_obj['milliseconds']

        # Output results to new file
        file = open(self.filename[:-5] + "-test.txt", "w")
        file.write('Abnormal pedestrian count: ' + str(abn_ped_count) + '\n')
        file.write('Negative time: ' + str(gnss_neg_time + trip_obj_neg_time) + '\n')
        if gnss_neg_time + trip_obj_neg_time > 0:
            file.write('\ttrip_gnss: ' + str(gnss_neg_time) + '\n')
            file.write('\ttrip_objects_forward: ' + str(trip_obj_neg_time) + '\n')
        file.write('Abnormal time jumps: ' + str(gnss_abn_jump + trip_obj_abn_jump) + '\n')
        if gnss_abn_jump + trip_obj_abn_jump > 0:
            file.write('\ttrip_gnss: ' + str(gnss_abn_jump) + '\n')
            file.write('\ttrip_objects_forward: ' + str(trip_obj_abn_jump) + '\n')
            if (gnss_abn_jump > 0):
                # Write GNSS time jump info to file
                file.write('\n\n')
                file.write('# Abnormal time jumps in GNSS data (milliseconds)\n')
                file.write('start_time, end_time, difference\n')
                for data in gnss_jump_list:
                    file.write(str(data[0]) + ', ' + str(data[1]) + ', ' + str(data[2]) + '\n')
        file.close()

    # Takes an x-value and two (x, y) pairs and returns the
    # estimated y-value from linear interpolation
    def lin_interpolation(self, x, pair1, pair2):
        x1 = float(pair1[0])
        y1 = float(pair1[1])

        x2 = float(pair2[0])
        y2 = float(pair2[1])

        y = y1 + (((y2 - y1) / (x2 - x1)) * (x - x1))
        return y

    # Takes a time from trip start in milliseconds and extracts
    # the corresponding vehicle dynamic data
    # Returns a list for given time
    def extract_vehicle_data(self, timestamp):
        initial_data = True
        last_time = last_speed = last_lat = last_long = last_alt = last_heading = -1
        last_fixmode = last_acc_horiz = last_acc_horiz = last_acc_speed = last_acc_heading = -1
        last_datetime = '-1'
        speed = lat = long = alt = heading = fixmode = acc_horiz = acc_vert = acc_speed = acc_heading = -1
        datetime = '-1'
        # Extract speed, -1 if no data for timestamp
        for gnss_obj in self.data['trip_gnss']:
            if initial_data:
                last_time = gnss_obj['milliseconds']
                last_speed = gnss_obj['speed']
                last_lat = gnss_obj['latitude']
                last_long = gnss_obj['longitude']
                last_alt = gnss_obj['altitude']
                last_heading = gnss_obj['heading']
                last_datetime = gnss_obj['datetime']
                last_fixmode = gnss_obj['fix_mode']
                last_acc_horiz = gnss_obj['accuracy_horizontal']
                last_acc_vert = gnss_obj['accuracy_vertical']
                last_acc_speed = gnss_obj['accuracy_speed']
                last_acc_heading = gnss_obj['accuracy_heading']
                initial_data = False
            else:
                if timestamp < gnss_obj['milliseconds'] and timestamp > last_time:
                    # Use linear interpolation to estimate speed
                    pair1 = [last_time, last_speed]
                    pair2 = [gnss_obj['milliseconds'], gnss_obj['speed']]
                    speed = self.lin_interpolation(timestamp, pair1, pair2)

                    pair1 = [last_time, last_lat]
                    pair2 = [gnss_obj['milliseconds'], gnss_obj['latitude']]
                    lat = self.lin_interpolation(timestamp, pair1, pair2)

                    pair1 = [last_time, last_long]
                    pair2 = [gnss_obj['milliseconds'], gnss_obj['longitude']]
                    long = self.lin_interpolation(timestamp, pair1, pair2)

                    pair1 = [last_time, last_alt]
                    pair2 = [gnss_obj['milliseconds'], gnss_obj['altitude']]
                    alt = self.lin_interpolation(timestamp, pair1, pair2)

                    pair1 = [last_time, last_heading]
                    pair2 = [gnss_obj['milliseconds'], gnss_obj['heading']]
                    heading = self.lin_interpolation(timestamp, pair1, pair2)

                    datetime = gnss_obj['datetime']

                    pair1 = [last_time, last_fixmode]
                    pair2 = [gnss_obj['milliseconds'], gnss_obj['fix_mode']]
                    fixmode = self.lin_interpolation(timestamp, pair1, pair2)

                    pair1 = [last_time, last_acc_horiz]
                    pair2 = [gnss_obj['milliseconds'], gnss_obj['accuracy_horizontal']]
                    acc_horiz = self.lin_interpolation(timestamp, pair1, pair2)

                    pair1 = [last_time, last_acc_vert]
                    pair2 = [gnss_obj['milliseconds'], gnss_obj['accuracy_vertical']]
                    acc_vert = self.lin_interpolation(timestamp, pair1, pair2)

                    pair1 = [last_time, last_acc_speed]
                    pair2 = [gnss_obj['milliseconds'], gnss_obj['accuracy_speed']]
                    acc_speed = self.lin_interpolation(timestamp, pair1, pair2)

                    pair1 = [last_time, last_acc_heading]
                    pair2 = [gnss_obj['milliseconds'], gnss_obj['accuracy_heading']]
                    acc_heading = self.lin_interpolation(timestamp, pair1, pair2)
                elif timestamp == gnss_obj['milliseconds']:
                    speed = gnss_obj['speed']
                    lat = gnss_obj['latitude']
                    long = gnss_obj['longitude']
                    alt = gnss_obj['altitude']
                    heading = gnss_obj['heading']
                    datetime = gnss_obj['datetime']
                    fixmode = gnss_obj['fix_mode']
                    acc_horiz = gnss_obj['accuracy_horizontal']
                    acc_vert = gnss_obj['accuracy_vertical']
                    acc_speed = gnss_obj['accuracy_speed']
                    acc_heading = gnss_obj['accuracy_heading']
                elif timestamp < gnss_obj['milliseconds']:
                    # We passed target time, exit search
                    break

                last_time = gnss_obj['milliseconds']
                last_speed = gnss_obj['speed']
                last_lat = gnss_obj['latitude']
                last_long = gnss_obj['longitude']
                last_alt = gnss_obj['altitude']
                last_heading = gnss_obj['heading']
                last_datetime = gnss_obj['datetime']
                last_fixmode = gnss_obj['fix_mode']
                last_acc_horiz = gnss_obj['accuracy_horizontal']
                last_acc_vert = gnss_obj['accuracy_vertical']
                last_acc_speed = gnss_obj['accuracy_speed']
                last_acc_heading = gnss_obj['accuracy_heading']

        initial_data = True
        last_time = last_acc_x = last_acc_y = last_yaw = last_yaw_dot = acc = yaw = yaw_dot = -1
        # Extract acceleration, -1 if no data for timestamp
        for imu_obj in self.data['trip_imu']:
            if initial_data:
                last_time = imu_obj['milliseconds']
                last_acc_x = imu_obj['acceleration_x']
                last_acc_y = imu_obj['acceleration_y']
                last_yaw = imu_obj['yaw_rate']
                last_yaw_dot = imu_obj['yaw_rate_dot']
                initial_data = False
            else:
                if timestamp < imu_obj['milliseconds'] and timestamp > last_time:
                    # Linear interpolation on x acc. and y acc. separately
                    pair1 = [last_time, last_acc_x]
                    pair2 = [imu_obj['milliseconds'], imu_obj['acceleration_x']]
                    acc_x = self.lin_interpolation(timestamp, pair1, pair2)

                    pair1 = [last_time, last_acc_y]
                    pair2 = [imu_obj['milliseconds'], imu_obj['acceleration_y']]
                    acc_y = self.lin_interpolation(timestamp, pair1, pair2)

                    # Combine components
                    acc = math.hypot(acc_x, acc_y)

                    pair1 = [last_time, last_yaw]
                    pair2 = [imu_obj['milliseconds'], imu_obj['yaw_rate']]
                    yaw = self.lin_interpolation(timestamp, pair1, pair2)

                    pair1 = [last_time, last_yaw_dot]
                    pair2 = [imu_obj['milliseconds'], imu_obj['yaw_rate_dot']]
                    yaw_dot = self.lin_interpolation(timestamp, pair1, pair2)
                elif timestamp == imu_obj['milliseconds']:
                    acc = math.hypot(imu_obj['acceleration_x'], imu_obj['acceleration_y'])
                    yaw = imu_obj['yaw_rate']
                    yaw_dot = imu_obj['yaw_rate_dot']
                elif timestamp < imu_obj['milliseconds']:
                    # We passed target time, exit search
                    break

                last_time = imu_obj['milliseconds']
                last_acc_x = imu_obj['acceleration_x']
                last_acc_y = imu_obj['acceleration_y']
                last_yaw = imu_obj['yaw_rate']
                last_yaw_dot = imu_obj['yaw_rate_dot']

        event_marker = '-1'
        for marker in self.data['trip_markers']:
            if timestamp == marker['milliseconds']:
                event_marker = str(marker['event_marker'])
            elif timestamp < marker['milliseconds']:
                # We passed target time, exit search
                break

        # Extract range and range_rate
        # Assumption: timestamp will be from trip_objects_forward
        range = -1
        range_rate = -1
        for trip_obj in self.data['trip_objects_forward']:
            if timestamp == trip_obj['milliseconds']:
                range = trip_obj['range']
                range_rate = trip_obj['range_rate']
            elif timestamp < trip_obj['milliseconds']:
                # We passed target time, exit search
                break

        vehicle_data = [datetime, fixmode, lat, long, alt, speed, heading, acc_horiz, acc_vert, acc_speed, acc_heading,
        acc, yaw, yaw_dot, range, range_rate, event_marker]
        return vehicle_data

    # Iterates through each trip_objects_forward in JSON file,
    # keeps track of separate event time info and pedestrian related event time info
    def read_trip_events(self):
        ped_rel_events = 0
        start_end = []
        ped_start_end = []

        # List of lists [pedestrian count, start time, end time] for each event
        event_info = []
        # List of lists [start time, end time] for each pedestrian related event
        ped_event_times = []

        last_time = 0
        for trip_obj in self.data['trip_objects_forward']:
            self.current_time = trip_obj['milliseconds']
            if trip_obj['pedestrian_count'] != self.current_pedestrian_count:
                # init_data only 0 on first trip_obj
                if self.init_data == 0:
                    self.current_pedestrian_count = trip_obj['pedestrian_count']
                    self.num_events += 1
                    self.start_time = trip_obj['milliseconds']
                    self.init_data = 1
                    last_time = trip_obj['milliseconds']

                    if trip_obj['pedestrian_count'] > 0:
                        ped_rel_events += 1
                        start_end.append(trip_obj['milliseconds'])
                else:
                    self.num_events += 1

                    if self.current_pedestrian_count == 0 and trip_obj['pedestrian_count'] > 0:
                        ped_rel_events += 1
                        start_end = []
                        start_end.append(trip_obj['milliseconds'])

                    if self.current_pedestrian_count > 0 and trip_obj['pedestrian_count'] == 0:
                        start_end.append(last_time)
                        ped_event_times.append(start_end)
                        start_end = []

                    ped_start_end.append(self.current_pedestrian_count)
                    self.current_pedestrian_count = trip_obj['pedestrian_count']

                    ped_start_end.append(self.start_time)
                    self.start_time = trip_obj['milliseconds']

                    ped_start_end.append(last_time)
                    last_time = trip_obj['milliseconds']

                    event_info.append(ped_start_end)
                    ped_start_end = []
            else:
                last_time = trip_obj['milliseconds']

        if len(start_end) == 1:
            start_end.append(self.current_time)
            ped_event_times.append(start_end)

        if self.num_events > 0:
            ped_start_end.append(self.current_pedestrian_count)
            ped_start_end.append(self.start_time)
            ped_start_end.append(self.current_time)
            event_info.append(ped_start_end)

        return ped_event_times

    def get_event_info(self):
        # 3D list of event_info for each pedestrian related event in this trip
        trip_event_data = []

        # event_info: List of lists [speed, acc, range, range_rate, time] for a
        # pedestrian related event
        event_info = []

        # ped_rel_events: List of 2-item lists [start_time, end_time]
        # for each pedestrian related event
        ped_rel_events = self.read_trip_events()

        # Targeting events that are >= 10 seconds (10000 milliseconds)
        # Remove events less than 10 seconds
        #ped_rel_events_long = [event for event in ped_rel_events if event[1] - event[0] >= 10000]

        # Targeting events that are 8 to 9 seconds
        #ped_rel_events_long = [event for event in ped_rel_events if event[1] - event[0] >= 8000 and event[1] - event[0] <= 9000]

        ped_event_count = 0
        #for times in ped_rel_events_long:
        for times in ped_rel_events:
            ped_event_count += 1
            start_time = times[0]
            end_time = times[1]

            for trip_obj in self.data['trip_objects_forward']:
                if trip_obj['milliseconds'] >= start_time and trip_obj['milliseconds'] <= end_time:
                    dyn_data = self.extract_vehicle_data(trip_obj['milliseconds'])
                    dyn_data.append(trip_obj['milliseconds'])
                    event_info.append(dyn_data)
                elif end_time < trip_obj['milliseconds']:
                    # We passed target time, exit search
                    break

            trip_event_data.append(event_info)
            event_info = []

        # Output data to new file
        file = open(self.filename[:-5] + "-data.txt", "w")
        #file = open(self.filename[:-5] + "-data-long.txt", "w")
        #file = open(self.filename[:-5] + "-data-8-9.txt", "w")
        file.write('# Pedestrian related event data (events 8-9 seconds, separated by blank lines)\n')
        file.write('# Number of pedestrian related events 8-9 seconds: ' + str(ped_event_count) + '\n')
        file.write('# milliseconds, datetime, fix_mode, lat, long, alt, speed, heading, accuracy_horiz, accuracy_vert, ' +
        'accuracy_speed, accuracy_heading, acc, yaw_rate, yaw_rate_dot, range, range_rate, event_marker\n\n')
        for event in trip_event_data:
            for data in event:
                file.write(str(data[17]) + ', ' + str(data[0]) + ', ' + str(data[1]) + ', ' + str(data[2]) + ', ' +
                str(data[3]) + ', ' + str(data[4]) + ', ' + str(data[5]) + ', ' + str(data[6]) + ', ' + str(data[7]) +
                ', ' + str(data[8]) + ', ' + str(data[9]) +', ' + str(data[10]) + ', ' + str(data[11]) + ', ' +
                str(data[12]) + ', ' + str(data[13]) + ', ' + str(data[14]) + ', ' + str(data[15]) + ', ' + str(data[16]) + '\n')
            file.write('\n')
        file.close()

# Output file with list of valid/invalid cases
def separate_valid_cases():
    valid_cases = []
    invalid_cases = []

    directory = '/Users/jackbehrend/UMich/UMTRI/Data/json-data/V54_3000-4000'
    for root, dirs, files in os.walk(directory):
        for filename in os.listdir(root):
            if filename.endswith("-test.txt"):
                print(filename[:-9] + ".json")
                valid = True
                f = open(root + '/' + filename, "r")
                if (f.readline().strip() != "Abnormal pedestrian count: 0"):
                    valid = False
                if (f.readline().strip() != "Negative time: 0"):
                    valid = False
                if (f.readline().strip() != "Abnormal time jumps: 0"):
                    valid = False
                f.close()

                if valid == True:
                    valid_cases.append(filename[:-9] + ".json")
                else:
                    invalid_cases.append(filename[:-9] + ".json")

    save_directory = '/Users/jackbehrend/UMich/UMTRI/Data/json-data'
    new_file = open(save_directory + "/verify_cases_3000-4000.txt", "w")
    new_file.write('# Valid Cases\n')
    for case in valid_cases:
        new_file.write(case + '\n')
    new_file.write('\n')
    new_file.write('# Invalid Cases\n')
    for case in invalid_cases:
        new_file.write(case + '\n')
    new_file.write('\n')
    new_file.close()

# Extract vehicle dynamic data and save each trip's information in
# a separate text file in its respective trip directory
def get_data():
    directory = '/Users/jackbehrend/UMich/UMTRI/Data/json-data/V54_3000-4000'
    for root, dirs, files in os.walk(directory):
        for filename in os.listdir(root):
            if filename.endswith(".json"):
                print(filename)
                path_to_file = str(root) + '/' + str(filename)
                reader = TripReader(path_to_file)
                reader.get_event_info()
                #reader.identify_abnormal_cases()

    """
    directory = '/Users/jackbehrend/UMich/UMTRI/Data/JSON'
    for filename in os.listdir(directory):
        if filename.endswith(".json"):
            print(filename)
            reader = TripReader(filename)
            #reader.get_event_info()
            #reader.identify_abnormal_cases()
            #reader.read_trip_events()
    """

# Output file with inconsistent start/end event times (w/ Huajian)
def check_times():
    directory = '/Users/jackbehrend/UMich/UMTRI/Data/json-data/V54_1000-2000'
    check_directory = '/Users/jackbehrend/UMich/UMTRI/Data/V54'
    invalid_files = []
    for root, dirs, files in os.walk(directory):
        for filename in os.listdir(root):
            if filename.endswith(".json"):
                path_to_file = str(root) + '/' + str(filename)
                reader = TripReader(path_to_file)
                times = reader.read_trip_events()
                for check_file in os.listdir(check_directory):
                    if check_file.endswith(filename + ".csv"):
                        print(check_file)
                        valid = True
                        event_count = 0
                        with open(check_directory + '/' + check_file, "r") as f:
                            for row in f:
                                if event_count > 0:
                                    start = row.split(',')[2]
                                    end = row.split(',')[3]
                                    if (int(times[event_count - 1][0]) != int(start) or int(times[event_count - 1][1]) != int(end)):
                                        valid = False
                                event_count += 1
                        f.close()

                        if valid == False:
                            invalid_files.append(filename)


    save_directory = '/Users/jackbehrend/UMich/UMTRI/Data/json-data'
    new_file = open(save_directory + "/verify_times_1000-2000.txt", "w")
    new_file.write('# Files with start/end time inconsistency\n')
    for file in invalid_files:
        new_file.write(file + '\n')
    new_file.close()

# Output list of trip files containing certain events (duration)
def get_long_cases():
    ten_sec_event_trips = []

    directory = '/Users/jackbehrend/UMich/UMTRI/Data/json-data/V54_3000-4000'
    for root, dirs, files in os.walk(directory):
        for filename in os.listdir(root):
            if filename.endswith("-data-long.txt"):
                print(filename[:-14] + ".json")
                has_ten_sec_event = False
                f = open(root + '/' + filename, "r")
                f.readline().strip()
                if (f.readline().strip() != "# Number of pedestrian related events >= 10 seconds: 0"):
                    has_ten_sec_event = True
                f.close()

                if has_ten_sec_event == True:
                    ten_sec_event_trips.append(filename[:-14] + ".json")

    save_directory = '/Users/jackbehrend/UMich/UMTRI/Data/json-data'
    new_file = open(save_directory + "/long_cases_3000-4000.txt", "w")
    new_file.write('# Trips with at least one event >= 10 seconds\n')
    for case in ten_sec_event_trips:
        new_file.write(case + '\n')
    new_file.close()

# Prints number of specified type of events in trip files
def get_num_events():
    total_num_events = 0

    directory = '/Users/jackbehrend/UMich/UMTRI/Data/json-data/V54_1-1000'
    for root, dirs, files in os.walk(directory):
        for filename in os.listdir(root):
            if filename.endswith("-data.txt"):
                print(filename[:-9] + ".json")
                f = open(root + '/' + filename, "r")
                f.readline().strip()
                this_num_events = int(f.readline().strip().split(': ')[1])
                total_num_events += this_num_events
                f.close()

    print(str(total_num_events))

# Prints information about total event duration and frequency to terminal
def get_duration():
    total_duration = 0
    event_count = 0
    long_event_count = 0

    directory = '/Users/jackbehrend/UMich/UMTRI/Data/json-data/V54_3000-4000'
    for root, dirs, files in os.walk(directory):
        for filename in os.listdir(root):
            if filename.endswith(".json"):
                print(filename)
                path_to_file = str(root) + '/' + str(filename)
                reader = TripReader(path_to_file)
                evs = reader.read_trip_events()

                for times in evs:
                    # Milliseconds
                    this_duration = times[1] - times[0]
                    if this_duration <= 147800:
                        total_duration += this_duration
                        event_count += 1
                        if this_duration >= 10000:
                            long_event_count += 1

    # Convert to minutes
    print(str(total_duration / 60000))
    print(str(event_count))
    print(str(long_event_count))

# Creates plot of certain data channels vs. time (milliseconds) and
# saves them or displays them
def create_graph():
    directory = '/Users/jackbehrend/UMich/UMTRI/Data/JSON'
    for filename in os.listdir(directory):
        if filename.endswith("00005-data.txt"):
            print(filename[:-9] + ".json")
            with open(filename, "r") as f:
                # List of event_coordinates lists
                events = []
                # List of coordinates [x, y] to graph
                event_coordinates = []

                for line in f:
                    if line[0] != '#':
                        if len(line.strip()) != 0:
                            # 0 => milliseconds
                            # 6 => speed
                            if float(line.strip().split(', ')[6]) != -1:
                                event_coordinates.append([float(line.strip().split(', ')[0]), float(line.strip().split(', ')[6])])
                        else:
                            if len(event_coordinates) > 0:
                                events.append(event_coordinates)
                            event_coordinates = []

                timestamps = []
                y_vals = []
                ev_count = 0
                for ev in events:
                    ev_count += 1
                    for points in ev:
                        timestamps.append(points[0])
                        y_vals.append(points[1])

                    plt.figure(figsize=(15, 5))
                    plt.plot(timestamps, y_vals)
                    plt.title(filename[:-9] + ".json Event " + str(ev_count))
                    plt.xlabel("Time (ms)")
                    plt.ylabel("Speed (m/s)")
                    plt.show()
                    #plt.savefig(filename[:-9] + "-" + ev_count + ".png")

                    timestamps = []
                    y_vals = []
            f.close()

def main():
    pass

    #get_data()
    #separate_valid_cases()
    #check_times()
    #get_long_cases()
    #get_num_events()
    #get_duration()
    #create_graph()

if __name__ == "__main__":
    main()
