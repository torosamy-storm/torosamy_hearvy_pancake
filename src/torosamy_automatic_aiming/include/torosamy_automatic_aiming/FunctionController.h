//
// Created by torosamy on 25-4-2.
//

#ifndef FUNCTIONCONTROLLER_H
#define FUNCTIONCONTROLLER_H

class FunctionController {
public:
    static void disableShowSrc();
    static void enableShowSrc();
private:
    FunctionController() = default;
    ~FunctionController() = delete;
    FunctionController(const FunctionController&) = delete;
    FunctionController& operator=(const FunctionController&) = delete;
    FunctionController(FunctionController&&) = delete;
    FunctionController& operator=(FunctionController&&) = delete;
};

#endif //FUNCTIONCONTROLLER_H
