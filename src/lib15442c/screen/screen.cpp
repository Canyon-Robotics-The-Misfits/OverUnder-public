#include "lib15442c/screen/screen.hpp"
#include "lib15442c/logger.hpp"
#include "config.hpp"
#include <iostream>

using namespace lib15442c;
using namespace pros::c;

void Screen::addButtons(std::vector<Button> newButtons)
{
    screen_mutex.lock();
    for (int i = 0; i < (int)newButtons.size(); i++)
    {
        this->buttons.push_back(newButtons[i]);
    }
    screen_mutex.unlock();
}

void Screen::render()
{
    int charWidth = 10;
    int charHeight = 15;
    screen_mutex.lock();
    for (int i = 0; i < (int)buttons.size(); i++)
    {
        if (i == selectedButton)
        {
            pros::screen::set_pen(buttons[i].enabledColor);
            pros::screen::set_eraser(buttons[i].enabledColor);
        }
        else
        {
            pros::screen::set_pen(buttons[i].disabledColor);
            pros::screen::set_eraser(buttons[i].disabledColor);
        }
        pros::screen::fill_rect(buttons[i].x, buttons[i].y, buttons[i].x + buttons[i].width, buttons[i].y + buttons[i].height);
        pros::screen::set_pen(pros::Color::white);
        if (buttons[i].id == ButtonIds::Calibrate && pros::c::imu_get_rotation(config::PORT_IMU) == PROS_ERR_F)
        {
            std::string calibratingText = "Calibrating";
            pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, ((2 * buttons[i].x + buttons[i].width) / 2) - (charWidth * calibratingText.length() / 2), (2 * buttons[i].y + buttons[i].height) / 2 - (charHeight / 2), "%s", calibratingText.c_str());
        }
        else
        {
            pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, ((2 * buttons[i].x + buttons[i].width) / 2) - (charWidth * buttons[i].text.length() / 2), (2 * buttons[i].y + buttons[i].height) / 2 - (charHeight / 2), "%s", buttons[i].text.c_str());
        }
        pros::screen::set_eraser(pros::Color::black);
    }
    screen_mutex.unlock();
}

int Screen::getButton(int x, int y)
{
    for (int i = 0; i < (int)buttons.size(); i++)
    {
        if (x >= buttons[i].x && x <= buttons[i].x + buttons[i].width && y >= buttons[i].y && y <= buttons[i].y + buttons[i].height)
        {
            return i;
        }
    }
    return -1;
}

Button Screen::getButtonByIndex(int index)
{
    screen_mutex.lock();
    Button temp = buttons[index];
    screen_mutex.unlock();
    return temp;
}

void Screen::setButtonEnabled(int index)
{
    screen_mutex.lock();
    selectedButton = index;
    screen_mutex.unlock();
}

Button Screen::getSelectedButton()
{
    screen_mutex.lock();
    if (selectedButton == -1)
    {
        screen_mutex.unlock();
        return Button(0, 0, 0, 0, "", pros::Color::black, pros::Color::black, ButtonIds::None);
    }
    Button temp = buttons[selectedButton];
    screen_mutex.unlock();
    return temp;
}

void Screen::registerScreenPressed()
{
    pros::screen::touch_callback([]()
                                 {
        pros::screen_touch_status_s_t status = pros::screen::touch_status();
        int buttonId = screen.getButton(status.x, status.y);
        if (buttonId == ButtonIds::Calibrate){
            pros::c::imu_reset(config::PORT_IMU);
        }
        else if (buttonId != -1) {
            screen.setButtonEnabled(buttonId);
        } },
                                 pros::E_TOUCH_PRESSED);
}

void Screen::startScreenThread()
{
    screen_task = pros::Task([this]
                             {
        int startingTime = pros::millis();
        int currentTime = pros::millis();
        pros::screen::erase();
        
        while (true) {
            render();
            currentTime = pros::millis();
            if (currentTime - startingTime > 5000 && selectedButton == -1 && competition_is_disabled()) {
                startingTime = currentTime;
                // master.rumble("..");
            } else if(competition_is_disabled()) {
                std::string selectedButtonText = screen.getSelectedButton().text;
                if(selectedButtonText.length() == 0){
                    selectedButtonText = "No Auto Selected";
                }
                // master.set_text(2, 0, selectedButtonText + "                             "); // Spaces clear the line
            }

            pros::delay(1000);
        } });
    screen_task.set_priority(TASK_PRIORITY_MIN);
}