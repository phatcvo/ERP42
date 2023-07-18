#include "comboboxmodel.h"
#include "qdebug.h"

comboboxmodel::comboboxmodel(QObject *parent) :
    QObject(parent)
{

}

comboboxmodel::comboboxmodel(const QStringList &list, int count, QObject *parent) :
    QObject(parent), m_comboList(list), m_count(count)
{

}

const QStringList comboboxmodel::comboList()
{
    return m_comboList;
}

void comboboxmodel::setComboList(const QStringList &comboList)
{

    if (m_comboList != comboList)
    {
        m_comboList = comboList;
        emit comboListChanged();
    }
}

int comboboxmodel::count()
{
    return m_count;
}

void comboboxmodel::setCount(int cnt)
{
    if (cnt != m_count)
    {
        m_count = cnt;
        emit countChanged();
    }
}

int comboboxmodel::currentIndex()
{
    return m_currentIndex;
}

void comboboxmodel::setcurrentIndex(int index)
{
    if (index != m_currentIndex)
    {
        m_currentIndex = index;
        qDebug()<<"current index is"<<index;
        qDebug()<<"current index is"<<m_comboList.at(index);
        emit currentIndexChanged();
    }
}
void comboboxmodel::addElement(const QString &element)
{
    m_comboList.append(element);
    emit comboListChanged();
    setCount(m_comboList.count());
    emit countChanged();

    for (int i = 0; i<m_count; i++)
    {
        qDebug() << m_comboList.at(i);
    }
}

void comboboxmodel::removeElement(int index)
{
    if (index < m_comboList.count())
    {
        m_comboList.removeAt(index);
        emit comboListChanged();
        setCount(m_comboList.count());
        emit countChanged();
    }

    for (int i = 0; i<m_count; i++)
    {
        qDebug() << m_comboList.at(i);
    }
}
