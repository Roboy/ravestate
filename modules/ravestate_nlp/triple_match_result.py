class TripleMatchResult:

    def __init__(self):
        self.preds = ()
        self.subs = ()
        self.objs = ()

    def __len__(self):
        return len(self.preds) + len(self.subs) + len(self.objs)

    def __str__(self):
        return 'preds: ' + str(self.preds) + ', subs: ' + str(self.subs) + ', objs: ' + str(self.objs)