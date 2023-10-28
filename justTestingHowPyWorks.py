class test:
    def __init__(self,):
        self.speed = 10

    def func(self, speed):
        print(speed)
        print(self.speed)

x = test()
x.func(30)