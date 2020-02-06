import numpy as np
# from Helper import Helper
class PointApproximator:
    """
    Helper class that holds feeded cooordinates 
    and approximates them after filling the buffer
    """
    def __init__(self, buffer_size = 20):
        """
        Keyword Arguments:
            buffer_size {int} -- [How many frames will be considered for approximation] (default: {20})
        """
        self.approximated_route = []
        self.BUFFER_SIZE = buffer_size
        self.data_buffer = np.zeros(self.BUFFER_SIZE, dtype=(int, 2))
        self.filled_counter = 0
        self.reset = 0
        self.finished_updating = False


    def update(self, coordinates) -> bool:
        """
        Provide tuple of coordinates that will be added to
        a list and approximated after filling the buffer
        
        Arguments:
            coordinates {(int, 2)} -- x, y coordinates
        """
        print(self.filled_counter)
        self.filled_counter += 1
        if(self.filled_counter % self.BUFFER_SIZE == 0):
            self.filled_counter = 0
            self.approximated_route.append(self.__approximate_last_position())
            print("APPROXIMATED HOST POSITION", self.approximated_route[-1], len(self.approximated_route))
            return True
        self.data_buffer[self.filled_counter] = coordinates


    def __approximate_last_position(self):
        self.reset += 1
        length = self.data_buffer.shape[0]
        sum_x = np.sum(self.data_buffer[:, 0])
        sum_y = np.sum(self.data_buffer[:, 1])
        return sum_x/length, sum_y/length

    def get_last_approximated_position(self):
        return self.approximated_route[-1]
        
    def get_last_n_approximated_positions(self, how_many):
        return self.approximated_route[-how_many:]

    # def get_last_car_yaw(self):
    #     last_position = self.get_last_n_approximated_positions(2)
    #     return Helper.angle_between_two_points(last_position[1], last_position[0])

    def finalize(self):
        return self.data_buffer
