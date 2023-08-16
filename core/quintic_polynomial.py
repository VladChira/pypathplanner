class QuinticPolynomial:
    def __init__(self, *args):
        if len(args) == 6:
            self.a = args[0]
            self.b = args[1]
            self.c = args[2]
            self.d = args[3]
            self.e = args[4]
            self.f = args[5]
        elif len(args[0]) == 6:
            self.a = args[0][0]
            self.b = args[0][1]
            self.c = args[0][2]
            self.d = args[0][3]
            self.e = args[0][4]
            self.f = args[0][5]
        else:
            raise ValueError(
                "Invalid polynomial coefficients given. Expected either six separate coefficients or an array of six coefficients.")

    def eval(self, t):
        return self.a * t**5 + self.b * t**4 + self.c * t**3 + self.d * t**2 + self.e * t + self.f

    def first_deriv(self):
        return QuinticPolynomial(0, 5 * self.a, 4 * self.b, 3 * self.c, 2 * self.d, self.e)

    def second_deriv(self):
        return QuinticPolynomial(0, 0, 20 * self.a, 12 * self.b, 6 * self.c, 2 * self.d)

    def __str__(self):
        terms = []

        if self.a != 0:
            terms.append(f"{round(self.a, 4)}t^5")

        if self.b != 0:
            terms.append(f"{round(self.b, 4)}t^4")

        if self.c != 0:
            terms.append(f"{round(self.c, 4)}t^3")

        if self.d != 0:
            terms.append(f"{round(self.d, 4)}t^2")

        if self.e != 0:
            terms.append(f"{round(self.e, 4)}t")

        if self.f != 0:
            terms.append(str(round(self.f, 4)))

        polynomial = " + ".join(terms[::-1])
        return polynomial