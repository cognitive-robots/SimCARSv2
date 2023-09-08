/*
 * Originally copied from https://www.sfml-dev.org/tutorials/1.6/graphics-qt.php on 04/06/2021, with permission from the original author.
 * Modified by Rhys Howard.
 * SFML is licensed under the terms and conditions of the zlib/png license.
 * Copyright © Laurent Gomila
 */

#include <ori/simcars/visualisation/qsfml_canvas_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace visualisation
{

AQSFMLCanvas::AQSFMLCanvas(QWidget *parent, QPoint const &position, QSize const &size,
                           std::chrono::milliseconds render_interval)
    : QWidget(parent), render_timer(this), initialised(false)
{
    // Setup some states to allow direct rendering into the widget
    setAttribute(Qt::WA_PaintOnScreen);
    setAttribute(Qt::WA_OpaquePaintEvent);
    setAttribute(Qt::WA_NoSystemBackground);

    // Set strong focus to enable keyboard events to be received
    setFocusPolicy(Qt::StrongFocus);

    // Setup the widget geometry
    move(position);
    resize(size);

    // Setup the timer
    render_timer.setInterval(render_interval);
}

QPaintEngine* AQSFMLCanvas::paintEngine() const
{
    return 0;
}

void AQSFMLCanvas::showEvent(QShowEvent *event)
{
    if (!initialised)
    {
        // Under X11, we need to flush the commands sent to the server to ensure that
        // SFML will get an updated view of the windows
        #ifdef Q_WS_X11
            XFlush(QX11Info::display());
        #endif

        // Create the SFML window with the widget handle
        sf::RenderWindow::create(winId());

        // Let the derived class do its specific stuff
        on_init();

        // Setup the timer to trigger a refresh at specified framerate
        connect(&render_timer, SIGNAL(timeout()), this, SLOT(repaint()));
        render_timer.start();

        initialised = true;
    }
}

void AQSFMLCanvas::paintEvent(QPaintEvent *event)
{
    // Let the derived class do its specific stuff
    on_update();

    // Display on screen
    display();
}

}
}
}
