#--------------Graph Helper Functions------------------
#These are vital graph functions used to create and apply soe processes on the graph
def addNodes(G,nodes):
    for i in nodes:
      G[i]=[]
    return G

def addEdges(G,edges,directed):
    if directed == True:            
        for i in edges:
            G[i[0]] = G[i[0]]+[(i[1],i[2])]
        return G
    elif directed==False:
        for i in edges:
            G[i[0]] = G[i[0]] + [(i[1],i[2])]
            G[i[1]] = G[i[1]] + [(i[0],i[2])]
            
        return G           



def listOfNodes(G):
    lst=[i for i in G.keys()]
    return lst




def listOfEdges(G, directed = False):
    lst = []
    for i in G.keys():
        for j in G[i]:
            if directed == False:
                lst.append((i,j[0],j[1]))
            elif directed == True:
                tem1 = (i,j[0],j[1])
                tem2 = (i,j[1],j[0])
                if tem1 and tem2 not in lst:
                    lst.append(tem1)
    return lst



def printIn_OutDegree(G):
    out_deg ={}
    in_deg ={}
    for i in G.keys():
        out_deg[i] = len(G[i])
        in_deg[i] = 0
    for i in G.values():
        for j in i:
            in_deg[j[0]] +=1
    #return in_deg

            
    for i in G.keys():
        print(i,"=> In-Degree: ", in_deg[i]," Out-Degree: ",out_deg[i])
        



def printDegree(G):
    for i in G.keys():
         print(i," => ",len(G[i]))


def getNeighbors(G,node):
    lst = []
    for i in G[node]:
        lst.append(i[0])
    return lst



def getInNeighbors(G,node):
    lst = []
    for i in G.keys():
        for j in G[i]:
            if j[0] == node:
                lst.append(i)
    return lst




def getOutNeighbors(G, node):
    lst = []
    for i in G[node]:
        lst.append(i[0])
    return lst


def getNearestNeighbor(G, node):
    short = float('inf')
    lst = G[node]
    near_node = -10
    for i in range(len(lst)):
        if lst[i][1]<short:
            short = lst[i][1]
            near_node = lst[i][0]
    return near_node

   
        

def isNeighbor(G,Node1,Node2):
    for i in G[Node1]:
        if i[0]==Node2:
            return True
    return False



def removeNode(G,node):
    G.pop(node)
    for i in G.keys():
        tem = []
        for j in G[i]:
            if j[0]!=node:
                tem.append(j)
        G[i] =tem
    return G

def removeNodes(G,nodes):
    for k in nodes:
        node = k
        G.pop(node)
        for i in G.keys():
            tem = []
            for j in G[i]:
                if j[0]!=node:
                    tem.append(j)
            G[i] =tem
        
    return G
            
def display_adj_matrix(G):
    lst = list(G.keys())
    zeros = [0 for i in range(len(lst))]
    fin_lst = []
    for i in G.keys():
        tem = zeros.copy()
        for j in G[i]:
            tem[lst.index(j[0])] = 1
        fin_lst.append(tem)

    for i in fin_lst:
        print(i)
    return fin_lst

#-------------Stack and Queue Functions--------------
def push(lst,x):
  return lst.append(x)

def pop(lst):
  return lst.pop()

def enqueue(lst,x):
  return lst.append(x)

def dequeue(lst):
  return lst.pop(0)

def is_empty(lst):
  if lst==[]:
    return True
  return False


#Priority queue for storing weighted edges accordingly
def p_enqueue(queue, priority, item):
    tup  = (priority,item)
    
    queue.append("*")
    length = 0
    while queue[length]!='*':
        length+=1
    queue.pop()
    
    counter = 0 
    index = 0
    
    while counter<length:
        if queue[counter][0]<=tup[0]:
            index +=1
        counter+=1
    queue.insert(index,tup)
    return queue
    

def Dequeue(queue):
    return queue.pop(0)[0]

#Dikstra's Algorith implementation using priority queues
def Dikstra(G,A):
  nodes    = listOfNodes(G)
  visited  = []
  distance = {}
  parents  = {}
  p_queue    = [[0,A]]
  for i in nodes:
    if i!=A:
      distance[i] = float('inf')
      parents[i]  = 'none'
    else:
      distance[i] = 0
      parents[i]  = i
      
  while p_queue!=[]:
    var = p_queue.pop(0)
    if var[1] not in visited:
      visited.append(var[1])
      for i in G[var[1]]:
        if (i[1]+distance[var[1]])<distance[i[0]]:
          distance[i[0]] =  i[1]+distance[var[1]]
          parents[i[0]]  = var[1]
          p_enqueue(p_queue,i[1]+distance[var[1]],i[0])
          #p_queue.append([i[1]+distance[var[1]],i[0]])
        
        '''
        print("---------------------------------------New Iter---------------------------------------")
        print(var[1])
        print(i[0])
        print(i[1])
        print(p_queue)
        print(visited)
        print(distance[var[1]])
        print(distance)
        '''
          
  return parents,distance


#Shortest path finding functions using results of dikstra
def shortestPath(G,A,B):
  (parents,distance) = Dikstra(G,A)
  var = B
  path  = []
  while var!=A:
    path = [(parents[var],var)] + path
    var = parents[var]
  return path
