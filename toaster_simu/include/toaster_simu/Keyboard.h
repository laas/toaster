/* 
 * File:   Keyboard.h
 * Author: gmilliez
 *
 * Created on December 28, 2015, 2:56 PM
 */

#ifndef KEYBOARD_H
#define	KEYBOARD_H

#include <SDL.h>

namespace keyboard {

    class Keyboard {
    public:
        Keyboard(void);
        ~Keyboard(void);

        bool get_key(bool& new_event, bool& pressed, uint16_t& code, uint16_t& modifiers);

    private:
        SDL_Surface* window;
    };
}

#endif