#!/usr/bin/env python
import copy
import modern_robotics
import rospy

def dfs(mapvals, checked, i, j):
	# right and down, no up and left
	print "dfs ", i, j
	resTmp = []
	checkedTmp = copy.deepcopy(checked)
	if (i+1 < len(mapvals)) and (mapvals[i+1][j] == 1) and ((i+1,j) not in checkedTmp):
		resTmp.append((i+1,j))
		checkedTmp.add((i+1,j))
		(innerRes, innerChecked) = dfs(mapvals, checkedTmp, i+1, j)
		checkedTmp = checkedTmp.union(innerChecked)
		if (innerRes != [] and innerChecked != []):
			resTmp += [innerRes]
	if ((i+1 < len(mapvals)) and (j+1 < len(mapvals[0]))) and (mapvals[i+1][j+1] == 1) and ((i+1,j+1) not in checkedTmp):
		resTmp.append((i+1,j+1))
		checkedTmp.add((i+1,j+1))
		(innerRes, innerChecked) = dfs(mapvals, checkedTmp, i+1, j+1)
		checkedTmp = checkedTmp.union(innerChecked)
		if (innerRes != [] and innerChecked != []):
			resTmp += [innerRes]
	if (j+1 < len(mapvals[0])) and (mapvals[i][j+1] == 1) and ((i, j+1) not in checkedTmp):
		resTmp.append((i,j+1))
		checkedTmp.add((i,j+1))
		(innerRes, innerChecked) = dfs(mapvals, checkedTmp, i, j+1)
		checkedTmp = checkedTmp.union(innerChecked)
		if (innerRes != [] and innerChecked != []):	
			resTmp += innerRes	
	return (resTmp, checkedTmp)

def getPointsFromEdges(edgemap):
	checked = set()
	res = []
	for i in xrange(len(edgemap)):
		for j in xrange(len(edgemap[i])):
			if ((i,j) not in checked) and (edgemap[i][j] == 1):
				checked.add((i,j))
				#res += [(i,j)]
				(resTmp, checkedTmp) = dfs(edgemap, checked, i, j)
				# add default value to say pick up end-effector
				checked = checked.union(checkedTmp)
				if (resTmp != []):
					res.append([(i,j)] + resTmp)
				else:
					res.append([(i,j)])
	print checked
	return res

def main():
        return 1

if __name__=='__main__':
    try:
        main()
    except KeyboardInterrupt:
        print ("Shutting Down")