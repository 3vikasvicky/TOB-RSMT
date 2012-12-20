result = file ('result.txt', 'w')

for i in range(1,10):
	filename = 'execution_RC0' + str(i) + '.log'
	result.write('RC0' + str(i) + '\n')
	f = file (filename, 'r')
	while True:
		line = f.readline();
		if not line:
			break
		if 'Min WNS = -' in line:
			result.write('WNS = ' + line[10:-1] + '\n')
	f.close()
	result.write('\n')

for i in range(10,13):
	filename = 'execution_RC' + str(i) + '.log'
	result.write('RC' + str(i) + '\n')
	f = file (filename, 'r')
	while True:
		line = f.readline();
		if not line:
			break
		if 'Min WNS = -' in line:
			result.write('WNS = ' + line[10:-1] + '\n')
	f.close()
	result.write('\n')

result.close()
		