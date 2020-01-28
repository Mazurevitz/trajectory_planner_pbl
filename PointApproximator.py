import numpy as np
class PointApproximator:
    """
    Helper class that holds feeded cooordinates 
    and approximates them after filling the buffer
    """
    def __init__(self):
        self.__approximated_route = []
        self.BUFFER_SIZE = 100
        self.data_buffer = np.zeros(self.BUFFER_SIZE, dtype=(int, 2))
        self.filled_counter = 0
        self.reset = 0


    def update(self, coordinates) -> bool:
        """
        Provide tuple of coordinates that will be added to
        a list and approximated after filling the buffer
        
        Arguments:
            coordinates {(int, 2)} -- x, y coordinates
        """
        self.filled_counter += 1
        if(self.filled_counter % self.BUFFER_SIZE == 0):
            self.filled_counter = 0
            self.__approximated_route.append(self.__approximate_last_position())
            print("APPROX", self.__approximated_route[-1],
            len(self.__approximated_route))
            return True
        self.data_buffer[self.filled_counter] = coordinates


    def __approximate_last_position(self):
        self.reset += 1
        length = self.data_buffer.shape[0]
        sum_x = np.sum(self.data_buffer[:, 0])
        sum_y = np.sum(self.data_buffer[:, 1])
        return sum_x/length, sum_y/length

    def get_last_position(self):
        return self.__approximated_route[-1]

    def finalize(self):
        return self.data_buffer
