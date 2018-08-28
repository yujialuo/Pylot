
import dos.graph as graph


class Env(object):
    def __init__(self):
        self.graph = graph.get_current_graph()

    def run(self):
        self.graph.run()
