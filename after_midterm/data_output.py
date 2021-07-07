
with open('myfile.txt','r') as file:
    linesList = file.readlines()
    first = linesList[0]
    last = linesList[-1]
file.close()

print("first line: ",first)
print("last line: ", last)
