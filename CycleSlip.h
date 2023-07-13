//
// Created by 0-0 mashuo on 2023/5/6.
//

#ifndef RTK_CYCLESLIP_H
#define RTK_CYCLESLIP_H

#include "vector"
#include "Matrix.h"

class CycleSlip{
public:
    std::vector<int> m_PRN[2];
    Matrix m_ObsMatrix[2];

public:

};

#endif //RTK_CYCLESLIP_H
