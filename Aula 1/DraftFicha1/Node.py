# Classe nodo para definiçao dos nodos
# cada nodo tem um nome e um id, poderia ter também um apontador para outro elemento a guardar....
class Node:
    def __init__(self, name, id=-1):     #  construtor do nodo....." id=-1 se nap houver parametro id ele mete -1 automaticamente
        self.m_id = id
        self.m_name = str(name)
        # posteriormente podera ser colocodo um objeto que armazena informação em cada nodo.....

    def __str__(self):
        return "node " + self.m_name

    def __repr__(self):
        return "node " + self.m_name

    def setId(self, id):
        self.m_id = id

    def getId(self):
        return self.m_id

    def getName(self):
        return self.m_name

    def __eq__(self, other):
        return self.m_name == other.m_name  # ver se é preciso tb testar o id....

    def __hash__(self):
        return hash(self.m_name)
