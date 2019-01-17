from scientio.ontology.node import Node


class DummySession:
    """
    Dummy Session that will be used, when Neo4j is unavailable.
    """

    def __init__(self, *args, **kwargs):
        pass

    def create(self, request: Node):
        return request

    def retrieve(self, *args, **kwargs):
        return []

    def update(self, request: Node) -> Node:
        return request

    def delete(self, *args, **kwargs) -> bool:
        return False