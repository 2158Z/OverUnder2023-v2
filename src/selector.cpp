#include "main.h"
#include "selector.h"
namespace selector{
    int auton;
    int autonCount;
    const char *btnMap[] = {"","","","","","","","","","",""};

    lv_obj_t *tabView;
    lv_obj_t *redBtn;
    lv_obj_t *blueBtn;

    lv_style_t relButtonStyle; //released style
    lv_style_t prButtonStyle; //pressed style

    lv_obj_t *Pspinbox;

    //Sets auton value to red side auton
    lv_res_t redBtnAction(lv_obj_t *btn, const char *txt){
        for(int i = 0; i < autonCount; i++){
            if(strcmp(txt, btnMap[i]) == 0){
                auton = i+1;
            }
        }

        return LV_RES_OK;
    }

    //Sets auton value to blue side auton
    lv_res_t blueBtnAction(lv_obj_t *btn, const char *txt){
        for(int i = 0; i < autonCount; i++){
            if(strcmp(txt, btnMap[i]) == 0){
                auton = -(i+1);
            }
        }

        return LV_RES_OK;
    }

    //Sets auton value to skills auton
    lv_res_t skillsBtnAction(lv_obj_t *btn){
        auton = 0;
        return LV_RES_OK;
    }

    int tabWatcher(){
        int activeTab = lv_tabview_get_tab_act(tabView);
        while(1){
            int currentTab = lv_tabview_get_tab_act(tabView);
            //Changes auton value to be valid for current tab selected
            if(currentTab != activeTab){
                activeTab = currentTab;
                switch(activeTab){
                    case 0:
                        if(auton == 0) auton = 1;
                        auton = abs(auton);
                        lv_btnm_set_toggle(redBtn, true, abs(auton)-1);
                        break;
                    case 1:
                        if(auton == 0) auton = -1;
                        auton = -abs(auton);
                        lv_btnm_set_toggle(blueBtn, true, abs(auton)-1);
                        break;
                    case 2:
                        auton = 0;
                        break;
                    case 3:
                        auton=0;
                        break;
                    default:
                        break;
                }
            }
            pros::delay(20);
        }
    }

    void init(int defaultAuton, const char **autons){
        //Map autons to buttons array
        int i = 0;
        do{
            memcpy(&btnMap[i], &autons[i], sizeof(&autons));
            i++;
        }while(strcmp(autons[i], "") != 0);

        autonCount = i;
        auton = defaultAuton;

        //Set theme to yellow
        lv_theme_t *th = lv_theme_alien_init(60, NULL);
        lv_theme_set_current(th);

        //Create tabview
        tabView = lv_tabview_create(lv_scr_act(), NULL);

        //Initiate Tabs
        lv_obj_t *redTab = lv_tabview_add_tab(tabView, "Red");
        lv_obj_t *blueTab = lv_tabview_add_tab(tabView, "Blue");
        lv_obj_t *skillsTab = lv_tabview_add_tab(tabView, "Skills");

        //Check if default auton is at a different tab and change tabs
        if(auton < 0){
            lv_tabview_set_tab_act(tabView, 1, LV_ANIM_NONE);
        }else if(auton == 0){
            lv_tabview_set_tab_act(tabView, 2, LV_ANIM_NONE);
        }

        //Create Red Button Tab
        redBtn = lv_btnm_create(redTab, NULL);
        lv_btnm_set_map(redBtn, btnMap);
        lv_btnm_set_action(redBtn, *redBtnAction);
        lv_btnm_set_toggle(redBtn, true, abs(auton)-1);
        lv_obj_set_size(redBtn, 450, 50);
        lv_obj_set_pos(redBtn, 0, 100);
        lv_obj_align(redBtn, NULL, LV_ALIGN_CENTER, 0, 0);

        //Create Blue Button Tab
        blueBtn = lv_btnm_create(blueTab, NULL);
        lv_btnm_set_map(blueBtn, btnMap);
        lv_btnm_set_action(blueBtn, *blueBtnAction);
        lv_btnm_set_toggle(blueBtn, true, abs(auton)-1);
        lv_obj_set_size(blueBtn, 450, 50);
        lv_obj_set_pos(blueBtn, 0, 100);
        lv_obj_align(blueBtn, NULL, LV_ALIGN_CENTER, 0, 0);

        //Create Skills Button
        lv_obj_t *skillsBtn = lv_btn_create(skillsTab, NULL);
        lv_obj_t *skillLabel = lv_label_create(skillsBtn, NULL);
        lv_label_set_text(skillLabel, "Skills");
        lv_btn_set_action(skillsBtn, LV_BTN_ACTION_CLICK, *skillsBtnAction);
        lv_obj_set_size(skillsBtn, 450, 50);
        lv_obj_set_pos(skillsBtn, 0, 100);
        lv_obj_align(skillsBtn, NULL, LV_ALIGN_CENTER, 0, 0);

        pros::Task tabWatcher_task(tabWatcher);
}