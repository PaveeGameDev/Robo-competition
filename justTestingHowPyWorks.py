class test:
    def __init__(self,):
        self.speed = 10

    def func(self, speed):
        print(speed)
        print(self.speed)

    def func2(self, speed):
        self.func(speed)

x = test()
x.func2(speed = 30)
print(speed)
