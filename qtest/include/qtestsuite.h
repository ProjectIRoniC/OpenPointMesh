#include <QObject>
#include <vector>

#ifndef QTESTSUITE_H
#define QTESTSUITE_H



class QTestSuite : public QObject
{
    Q_OBJECT
public:
    static std::vector<QObject*> m_suites;

public:
    explicit QTestSuite();

};

#endif // QTESTSUITE_H
