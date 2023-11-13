f = open("text.txt", "r")
celek = []
for i in range(46):
    line = list(f.readline().split(", "))
    print(line)
    for i in range(len(line)):
        line[i] = int(line[i])
    line[1] = 1680 - line[1]
    celek.append(line)

fa = open("text.txt", "w")
fa.write(str(celek))
fa.close()