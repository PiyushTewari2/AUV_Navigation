def way_point_maker(path_edges):
    global path_list
    coorda = coord1  # Set the initial coordinate
    path_list = [coord1]  # Start with the initial coordinate

    head = initial_heading  # Set the initial heading

    # Iterate for each path segment
    for i in range(1, path_edges + 1):
        # Prompt the user for distance and angle
        dis = np.float64(input("Enter the length of " + str(i) + "th path in meters: "))
        thetha2 = np.float64(input("Enter the angle in degrees by which the next coordinate is with respect to the initial heading in a clockwise sense. For the 1st path, it will be 0: "))

        # Calculate the bearing based on the initial heading and the provided angle
        if thetha2 < 360 - head:
            bear = thetha2 + head
        elif thetha2 >= 360 - head:
            bear = thetha2 + head - 360
        elif thetha2 + head == 720:
            bear = 0

        bear = math.radians(bear)  # Convert the bearing to radians

        # Convert distance from meters to kilometers
        dis = dis / 1000

        # Calculate the coordinates of the next waypoint using inverse haversine
        coordb = inverse_haversine(coorda, dis, bear)

        # Add the coordinates to the path_list
        path_list.append(coordb)

        # Update coorda for the next iteration
        coorda = coordb
