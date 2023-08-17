def func(i):
    print(i)


my_funcs = []
for i in range(3):
    my_funcs.append(lambda i=i: func(i))

for my_func in my_funcs:
    my_func()



    