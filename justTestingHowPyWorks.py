##Math functions
def mod(a,b):
    if a < 0 and b > 0 or a > 0 and b < 0:
        return a % -b
    else:
        return a % b

print(mod(-1,5))
print(mod(-1,-5))
print(mod(1,-5))
print(mod(1,5))