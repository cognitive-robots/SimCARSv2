/*
 * Originally copied from https://www.sfml-dev.org/tutorials/1.6/graphics-qt.php on 04/06/2021, with permission from the original author.
 * Modified by Rhys Howard.
 * SFML is licensed under the terms and conditions of the zlib/png license.
 * Copyright © Laurent Gomila
 */

#include <SFML/Graphics.hpp>
#include <QWidget>
#include <QTimer>


namespace ori
{
namespace simcars
{
namespace visualisation
{

class AQSFMLCanvas : public QWidget, public sf::RenderWindow
{
    QTimer render_timer;
    bool initialised;

    QPaintEngine* paintEngine() const override;

    void showEvent(QShowEvent *event) override;
    void paintEvent(QPaintEvent *event) override;

protected:
    virtual void on_init() = 0;
    virtual void on_update() = 0;

public :
    AQSFMLCanvas(QWidget *parent, QPoint const &position, QSize const &size, int render_interval = 0);
};

}
}
}
