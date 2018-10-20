from modules.dialogic_verbaliser.qa_parser import QAParser


class Verbaliser:
    """
    Produces actual utterances. This should in the future lead to diversifying
    the ways Roboy is expressing information.
    """
    path_to_qalist = "" # get from config
    path_to_info_list = "" # get from config
    qa_list_parser = None
    info_list_parser = None

    def __init__(self):
        self.qa_list_parser = QAParser(self.path_to_qalist)
        self.info_list_parser = QAParser(self.path_to_info_list)
