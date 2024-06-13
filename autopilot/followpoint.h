/*
 *     Copyright 2024 Rickard Häll   rickard.hall@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Follow a person or other vehicle when the point to follow is continously updated.
 */

#ifndef FOLLOWPOINT_H
#define FOLLOWPOINT_H

#include <QObject>

class FollowPoint : public QObject
{
    Q_OBJECT
public:
    explicit FollowPoint(QObject *parent = nullptr);

signals:

};

#endif // FOLLOWPOINT_H
