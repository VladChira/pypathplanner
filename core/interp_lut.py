class InterpLUT:
    def __init__(self, x, y):
        self.x_values = x
        self.y_values = y

    def lerp(self, x, x0, x1, y0, y1):
        return y0 + (x - x0) * (y1 - y0) / (x1 - x0)

    def lookup(self, x):
        if x <= self.x_values[0]:
            return self.y_values[0]
        if x >= self.x_values[-1]:
            return self.y_values[-1]

        idx = 0
        while x > self.x_values[idx]:
            idx += 1

        x0, x1 = self.x_values[idx - 1], self.x_values[idx]
        y0, y1 = self.y_values[idx - 1], self.y_values[idx]

        return self.lerp(x, x0, x1, y0, y1)