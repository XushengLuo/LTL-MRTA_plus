import sys


class Task(object):
    def __init__(self):
        # self.formula = '<> l1_1_1_0 && <> l2_2_1_0 && <> l3_3_1_0 && <> l4_1_1_0 && <> l5_2_1_0 && <> l6_3_1_0 && <> l7_1_1_0 ' \
        #                '&& <> l8_2_1_0 && <> l9_3_1_0 && <> l10_1_1_0'
        # n = int(sys.argv[1])//3
        # self.formula = '[] <> l1_1_{0}_0 && []<> l2_2_{0}_0 && [] <> l3_3_{0}_0  && []<> l4_4_{0}_0 && [] <> l5_5_{0}_0 && ' \
        #                '<> [] l6_1_{0}_0 && <> [] l7_2_{0}_0 && <>[] l8_3_{0}_0 && <> [] l9_4_{0}_0 && <>[] l10_5_{0}_0 '.format(n)


        # self.formula = '[] <> l1_1_{0}_0 && <> [] l6_1_{0}_0'.format(n)
        # self.formula = '[] <> l3_2_2 && [] <> l4_1_2 && [] <> l5_3_2  && [] <> l6_1_3 && []<> l7_1_3 && ' \
        #                '[]<> l1_2_2 && []<> l2_3_2'
        # sel f.formula = '[] <> l6_1_2 && []<> l4_1_2 && []<> l2_1_2'
        # && <> l5_2_1 && <> l6_3_1 && <> l7_1_1 ' \
        #                '&& <> l8_2_1 && <> l9_3_1 && <> l10_1_1'
        # self.formula = '<> l1_1_1 && <> l2_2_1 && <> ((l3_1_1 && l2_1_1) || l3_2_1)  && <> l4_1_1 && <> l5_1_1'
        # self.formula = '<> (l1_1_1 && <> l2_1_1) && <> ((l3_1_1 && l5_1_2) || l4_1_1))'
        # self.formula = '<> (l1_1_3 && X <> l2_2_4) && <> (l3_1_3  && l4_3_2)'
        # self.formula = '<> (l1_1_1 && X <> (l2_1_2 && X <> l3_2_1))'
        # self.formula = '<> (l1_1_1_0 && l5_3_1_0) && <> l2_2_2_0 && <> l4_3_2_0'
        # self.ap = [[['l1', 1, 1], ['l5', 3, 1]], [['l2', 2, 2]], [['l4', 3, 2]]]
        # self.formula = '[]<> (l1_1_2 && X<> l2_1_1) && <> l3_1_1'
        # self.formula = '[]<> (l1_1_1 && <> l2_1_1) && <> [] l3_1_1'
        # self.formula = '[]<> (l1_1_1 && <> l2_1_1) && [] <> ((l3_1_1 && l5_1_2) || l4_1_1))'
        # self.formula = '[] <> (l1_1_1 && <> ((l2_1_1 && l2_1_1) || l3_2_1)) && <> [] l2_2_1'
        # self.formula = '[] <> l1_1_1 && [] <> l2_2_1 '
        # self.formula = '[]<> (l1_1_1_0 && <> l2_1_1_0)'
        # self.formula = '<> (l1_1_2 && X <> l3_1_1) && <> l2_2_1'
        # self.formula = '[]<> l1_1_2 && <> (l2_2_2 || l3_2_2) && []<> l3_1_2'
        # self.formula = '[] <> l1_1_1_0 && <> l3_1_1_0  && [] <> (l2_2_1_0 || l3_2_1_0) '  # ok
        # self.formula = '[]<> l1_1_1_1 && []<> l3_1_1_1 && []<> (l2_2_1_0 || l3_2_1_0) '  # not ok
        # self.formula = '[] <> l1_1_1_1 && <> l3_1_1_1  && [] <> (l3_2_1_0) '  # ok
        # self.formula = '[] <> l1_1_1_1 && [] <> (l2_2_1_0 || l3_2_1_0) '  # not ok

        # self.formula = '[] <> (l2_2_1_0 || l3_2_1_0)'  # ok

        # case 3
        self.formula = '[]<> (l1_1_3_1 && l4_4_4_0)  && []<> l2_3_3_2 && [] <> l3_1_3_1 && [] <> l5_3_3_2 && []<> l6_4_2_0' \
                       '&& <> (l7_4_3_0 && (l8_5_3_3 || l10_5_3_3) && X <> l9_5_3_3) && <>[] l10_5_1_0'
        self.ap = [[['l1', 1, 3], ['l4', 4, 4]], [['l2', 3, 3]], [['l3', 1, 3]], [['l5', 3, 3]],
                   [ ['l6', 4, 2]], [['l7', 4, 3], ['l8', 5, 3]], [['l7', 4, 3], ['l10', 5, 3]], [['l9', 5, 3]],
                   [['l10', 5, 1]]]

