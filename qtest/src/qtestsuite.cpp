#include "../include/qtestsuite.h"
#include <iostream>

std::vector<QObject*> QTestSuite::m_suites;

QTestSuite::QTestSuite() : QObject()
{
    m_suites.push_back(this);
}
