class Test:
    def __inti__(self):
        print("Initialized")

    def method1(self):
        print("This is method 1, calling upon method 2")
        self.method2()

