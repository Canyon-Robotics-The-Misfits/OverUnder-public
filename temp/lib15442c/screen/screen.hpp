#pragma once

#include "pros/colors.hpp"
#include "pros/screen.h"
#include "pros/rtos.hpp"
#include "pros/misc.hpp"
#include <string>
#include <vector>

namespace lib15442c {

enum ButtonIds {
    CloseAWP,
    CloseElims,
    CloseSide,
    FarAWP,
    FarElims,
    FarSide,
    Skills,
    Calibrate,
    None
};

class Button {
public:
    Button(int x, int y, int width, int height, std::string text, pros::Color disabledColor, pros::Color enabledColor, lib15442c::ButtonIds id) :
        x(x), y(y), width(width), height(height), text(text), disabledColor(disabledColor), enabledColor(enabledColor), id(id)
    {
    }

    int x;
    int y;
    int width;
    int height;
    std::string text;
    pros::Color disabledColor;
    pros::Color enabledColor;
    lib15442c::ButtonIds id;
};

class Screen {

public:
    Screen() { }

    std::vector<Button> buttons = {};
    int selectedButton = -1;

    pros::Controller master = pros::Controller(pros::E_CONTROLLER_MASTER);

    void render();

    int getButton(int x, int y);
    Button getButtonByIndex(int index);
    void setButtonEnabled(int index);
    Button getSelectedButton();

    void addButtons(std::vector<Button> newButtons);

    pros::Task screen_task = pros::Task([] { return; });
    pros::Mutex screen_mutex = pros::Mutex();
    void startScreenThread();

    void registerScreenPressed();
};
}

inline lib15442c::Screen screen;