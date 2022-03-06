#ifndef INC_7405XV5_DISPLAY_H
#define INC_7405XV5_DISPLAY_H

#ifdef __cplusplus
#include "pros/apix.h"

#include <iterator>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>

extern "C" {
#endif
void gui_initialize();
#ifdef __cplusplus
}

namespace lib7405x {
    class Serial;
}

namespace lib7405x::display {
    typedef enum team_e {
        BLUE = 0,
        RED = 1
    } team_e_t;

    /**
    * Takes current state of team button and returns it
    * @return boolean, true if on red team
    */
    team_e_t getTeam();

    void renderGUI();

    lv_res_t teamChange(lv_obj_t* button);

    void create_home_tab(lv_obj_t* parent);

    void create_config_tab(lv_obj_t* parent);

    void create_funni_tab(lv_obj_t* parent);

    void reloadConstants();

    class Terminal {
        friend ::lib7405x::Serial;

    private:
        constexpr static int LINE_MAX = 9;
        std::string terminal_text[LINE_MAX]{};
        uint32_t startIdx = 0;
        void updateText();
        void render();
        lv_obj_t* _parent{nullptr};
        lv_obj_t* lcd_text[LINE_MAX]{};

        /**
        * Enqueues a new line on the Serial page
        * @param str the text displayed on the line
        */
        void println(const std::string& str);

    public:
        Terminal() = default;
        ~Terminal() {
            if (_parent) {
                for (auto obj : lcd_text) {
                    delete obj;
                }
            }
        }
        void setParent(lv_obj_t* parent);
        static Terminal* serial;
        static Terminal* logs;
    };

}// namespace lib7405x::display
#endif
#endif//INC_7405XV5_DISPLAY_H
