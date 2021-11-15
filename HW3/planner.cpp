#include <iostream>
#include <fstream>
// #include <boost/functional/hash.hpp>
#include <regex>
#include <unordered_set>
#include <set>
#include <list>
#include <unordered_map>
#include <algorithm>
#include <stdexcept>

#include <queue>          // std::priority_queue
#include <vector>         // std::vector
#include <functional>     // std::greater

#include <string>

#include <float.h>
#include <limits.h>
#include <stdlib.h>
#include <chrono>
#include <stack>
#include <utility>   
#include <cstdint>
#include <cstring>
#include <cmath>
#include <assert.h>

#define SYMBOLS 0
#define INITIAL 1
#define GOAL 2
#define ACTIONS 3
#define ACTION_DEFINITION 4
#define ACTION_PRECONDITION 5
#define ACTION_EFFECT 6

class GroundedCondition;
class Condition;
class GroundedAction;
class Action;
class Env;

using namespace std;

bool print_status = true;

class GroundedCondition
{
private:
    string predicate;
    list<string> arg_values;
    
    bool truth = true;

public:
    GroundedCondition(string predicate, list<string> arg_values, bool truth = true)
    {
        this->predicate = predicate;
        this->truth = truth;  // fixed
        for (string l : arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    GroundedCondition(const GroundedCondition& gc)
    {
        this->predicate = gc.predicate;
        this->truth = gc.truth;  // fixed
        for (string l : gc.arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }
    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    void set_truth(bool input){
        this->truth = input;
    }

    friend ostream& operator<<(ostream& os, const GroundedCondition& pred)
    {
        os << pred.toString() << " ";
        return os;
    }

    bool operator==(const GroundedCondition& rhs) const
    {
        if (this->predicate != rhs.predicate || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth()) // fixed
            return false;

        return true;
    }

    string toString() const
    {
        string temp = "";
        temp += this->predicate;
        temp += "(";
        for (string l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct GroundedConditionComparator
{
    bool operator()(const GroundedCondition& lhs, const GroundedCondition& rhs) const
    {
        return lhs == rhs;
    }
};

struct GroundedConditionHasher
{
    size_t operator()(const GroundedCondition& gcond) const
    {
        return hash<string>{}(gcond.toString());
    }
};

class Condition
{
private:
    string predicate;
    list<string> args;
    bool truth;

public:
    Condition(string pred, list<string> args, bool truth)
    {
        this->predicate = pred;
        this->truth = truth;
        for (string ar : args)
        {
            this->args.push_back(ar);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }

    list<string> get_args() const
    {
        return this->args;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    friend ostream& operator<<(ostream& os, const Condition& cond)
    {
        os << cond.toString() << " ";
        return os;
    }

    bool operator==(const Condition& rhs) const // fixed
    {

        if (this->predicate != rhs.predicate || this->args.size() != rhs.args.size())
            return false;

        auto lhs_it = this->args.begin();
        auto rhs_it = rhs.args.begin();

        while (lhs_it != this->args.end() && rhs_it != rhs.args.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth())
            return false;

        return true;
    }

    string toString() const
    {
        string temp = "";
        if (!this->truth)
            temp += "!";
        temp += this->predicate;
        temp += "(";
        for (string l : this->args)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ConditionComparator
{
    bool operator()(const Condition& lhs, const Condition& rhs) const
    {
        return lhs == rhs;
    }
};
// jia zhao, phd shen qing, hw3, exam, paper, Statement of Purpose, recommendation, cs Master. crazyflie(), planning project gg
struct ConditionHasher
{
    size_t operator()(const Condition& cond) const
    {
        return hash<string>{}(cond.toString());
    }
};

class Action
{
private:
    string name;
    list<string> args;
    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;

public:
    Action(string name, list<string> args,
        unordered_set<Condition, ConditionHasher, ConditionComparator>& preconditions,
        unordered_set<Condition, ConditionHasher, ConditionComparator>& effects)
    {
        this->name = name;
        for (string l : args)
        {
            this->args.push_back(l);
        }
        for (Condition pc : preconditions)
        {
            this->preconditions.insert(pc);
        }
        for (Condition pc : effects)
        {
            this->effects.insert(pc);
        }
    }
    string get_name() const
    {
        return this->name;
    }
    list<string> get_args() const
    {
        return this->args;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_preconditions() const
    {
        return this->preconditions;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_effects() const
    {
        return this->effects;
    }

    bool operator==(const Action& rhs) const
    {
        if (this->get_name() != rhs.get_name() || this->get_args().size() != rhs.get_args().size())
            return false;

        return true;
    }

    friend ostream& operator<<(ostream& os, const Action& ac)
    {
        os << ac.toString() << endl;
        os << "Precondition: ";
        for (Condition precond : ac.get_preconditions())
            os << precond;
        os << endl;
        os << "Effect: ";
        for (Condition effect : ac.get_effects())
            os << effect;
        os << endl;
        return os;
    }

    string toString() const
    {
        string temp = "";
        temp += this->get_name();
        temp += "(";
        for (string l : this->get_args())
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ActionComparator
{
    bool operator()(const Action& lhs, const Action& rhs) const
    {
        return lhs == rhs;
    }
};

struct ActionHasher
{
    size_t operator()(const Action& ac) const
    {
        return hash<string>{}(ac.get_name());
    }
};

class Env
{
private:
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> initial_conditions;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal_conditions;
    unordered_set<Action, ActionHasher, ActionComparator> actions;
    unordered_set<string> symbols;

public:
    void remove_initial_condition(GroundedCondition gc)
    {
        this->initial_conditions.erase(gc);
    }
    void add_initial_condition(GroundedCondition gc)
    {
        this->initial_conditions.insert(gc);
    }
    void add_goal_condition(GroundedCondition gc)
    {
        this->goal_conditions.insert(gc);
    }
    void remove_goal_condition(GroundedCondition gc)
    {
        this->goal_conditions.erase(gc);
    }
    void add_symbol(string symbol)
    {
        symbols.insert(symbol);
    }
    void add_symbols(list<string> symbols)
    {
        for (string l : symbols)
            this->symbols.insert(l);
    }
    void add_action(Action action)
    {
        this->actions.insert(action);
    }

    Action get_action(string name)
    {
        for (Action a : this->actions)
        {
            if (a.get_name() == name)
                return a;
        }
        throw runtime_error("Action " + name + " not found!");
    }
    unordered_set<string> get_symbols() const
    {
        return this->symbols;
    }

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_initConditions(){
        return this->initial_conditions;
    }

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_goalConditions(){
        return this->goal_conditions;
    }
    unordered_set<Action, ActionHasher, ActionComparator> get_actions(){
        return this->actions;
    }

    friend ostream& operator<<(ostream& os, const Env& w)
    {
        os << "***** Environment *****" << endl << endl;
        os << "Symbols: ";
        for (string s : w.get_symbols())
            os << s + ",";
        os << endl;
        os << "Initial conditions: ";
        for (GroundedCondition s : w.initial_conditions)
            os << s;
        os << endl;
        os << "Goal conditions: ";
        for (GroundedCondition g : w.goal_conditions)
            os << g;
        os << endl;
        os << "Actions:" << endl;
        for (Action g : w.actions)
            os << g << endl;
        cout << "***** Environment Created! *****" << endl;
        return os;
    }
};

class GroundedAction
{
private:
    string name;
    list<string> arg_values;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> preconditions;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> effects;

public:
    GroundedAction(string name, list<string> arg_values)
    {
        this->name = name;
        for (string ar : arg_values)
        {
            this->arg_values.push_back(ar);
        }
    }

    string get_name() const
    {
        return this->name;
    }

    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_preconditions() const
    {
        return this->preconditions;
    }
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_effects() const
    {
        return this->effects;
    }
    void set_preconditions(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> preconditions_in){
        this->preconditions = preconditions_in;
    }

    void set_effects(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> effects_in){
        this->effects = effects_in;
    }


    bool operator==(const GroundedAction& rhs) const
    {
        if (this->name != rhs.name || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }
        return true;
    }

    friend ostream& operator<<(ostream& os, const GroundedAction& gac)
    {
        os << gac.toString() << endl;
        os << "Precondition: ";
        for (GroundedCondition precond : gac.get_preconditions())
            os << precond;
        os << endl;
        os << "Effect: ";
        for (GroundedCondition effect : gac.get_effects())
            os << effect;
        os << endl;
        return os;
    }

    string toString() const
    {
        string temp = "";
        temp += this->name;
        temp += "(";
        for (string l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

list<string> parse_symbols(string symbols_str)
{
    list<string> symbols;
    size_t pos = 0;
    string delimiter = ",";
    while ((pos = symbols_str.find(delimiter)) != string::npos)
    {
        string symbol = symbols_str.substr(0, pos);
        symbols_str.erase(0, pos + delimiter.length());
        symbols.push_back(symbol);
    }
    symbols.push_back(symbols_str);
    return symbols;
}

Env* create_env(char* filename)
{
    ifstream input_file(filename);
    Env* env = new Env();
    regex symbolStateRegex("symbols:", regex::icase);
    regex symbolRegex("([a-zA-Z0-9_, ]+) *");
    regex initialConditionRegex("initialconditions:(.*)", regex::icase);
    regex conditionRegex("(!?[A-Z][a-zA-Z_]*) *\\( *([a-zA-Z0-9_, ]+) *\\)");
    regex goalConditionRegex("goalconditions:(.*)", regex::icase);
    regex actionRegex("actions:", regex::icase);
    regex precondRegex("preconditions:(.*)", regex::icase);
    regex effectRegex("effects:(.*)", regex::icase);
    int parser = SYMBOLS;

    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;
    string action_name;
    string action_args;

    string line;
    if (input_file.is_open())
    {
        while (getline(input_file, line))
        {
            string::iterator end_pos = remove(line.begin(), line.end(), ' ');
            line.erase(end_pos, line.end());

            if (line == "")
                continue;

            if (parser == SYMBOLS)
            {
                smatch results;
                if (regex_search(line, results, symbolStateRegex))
                {
                    line = line.substr(8);
                    sregex_token_iterator iter(line.begin(), line.end(), symbolRegex, 0);
                    sregex_token_iterator end;

                    env->add_symbols(parse_symbols(iter->str()));  // fixed

                    parser = INITIAL;
                }
                else
                {
                    cout << "Symbols are not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == INITIAL)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, initialConditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_initial_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_initial_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = GOAL;
                }
                else
                {
                    cout << "Initial conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == GOAL)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, goalConditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_goal_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_goal_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = ACTIONS;
                }
                else
                {
                    cout << "Goal conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTIONS)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, actionRegex))
                {
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Actions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_DEFINITION)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, conditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;
                    // name
                    action_name = iter->str();
                    iter++;
                    // args
                    action_args = iter->str();
                    iter++;

                    parser = ACTION_PRECONDITION;
                }
                else
                {
                    cout << "Action not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_PRECONDITION)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, precondRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition precond(predicate, parse_symbols(args), truth);
                        preconditions.insert(precond);
                    }

                    parser = ACTION_EFFECT;
                }
                else
                {
                    cout << "Precondition not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_EFFECT)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, effectRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition effect(predicate, parse_symbols(args), truth);
                        effects.insert(effect);
                    }

                    env->add_action(
                        Action(action_name, parse_symbols(action_args), preconditions, effects));

                    preconditions.clear();
                    effects.clear();
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Effects not specified correctly." << endl;
                    throw;
                }
            }
        }
        input_file.close();
    }

    else
        cout << "Unable to open file";

    return env;
}

class State{
public: 
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> conditions;
    State* parent = nullptr;
    GroundedAction* action = nullptr;

    double g = (double) DBL_MAX;
    double h = (double) DBL_MAX;
    double f = (double) DBL_MAX;


    State(){
        this->conditions = {};
    }

    State(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> c){
        this->conditions = c;
    }

    void setGval(double gVal){
        this->g = gVal;
    }

    void setHval(double hVal){
        this->h = hVal;
    }

    void updateFval(){
        this->f = this->g + this->h;
    }

    bool correctCondition(GroundedCondition gc) const {
        if ((this->conditions).find(gc) != (this->conditions).end()) {
            return true;
        }
        return false;
    }

    GroundedAction* get_GroundedAction(){
        return this->action;
    }
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_conditions(){
        return this->conditions;
    }

    void remove_condition(GroundedCondition gc) {
        gc.set_truth(true);
        this->conditions.erase(gc);
    }

    void add_condition(GroundedCondition gc) {
        this->conditions.insert(gc);
    }

    bool operator==(const State &rhs) const {
        bool result1 = true;
        bool result2 = true;
        for (GroundedCondition gc: rhs.conditions) {
            if (!(this->correctCondition(gc))) {
                result1 = false;
                break;
            }
        }
        for (GroundedCondition gc: this->conditions) {
            if (!(rhs.correctCondition(gc))) {
                result2 = false;
                break;
            }
        }
        return (result1 & result2);
    }
};

class Compare_State {
public:
    bool operator()(State *a, State *b) {
        return(a->f > b->f);
    }
};
    
    
double calc_Heuristic(State* tempState, State* goalState){
    double output = 0.0;
    for (auto itr = goalState->conditions.begin(); itr != goalState->conditions.end(); itr++){
        if (tempState->conditions.find(*itr) == tempState->conditions.end()){
            output += 1.0;
        }
    }
    return output;
}

bool at_goal(State* tempState, State* goalState){
    bool goal = true;
    for (auto itr = goalState->conditions.begin(); itr != goalState->conditions.end(); itr++){
        if (tempState->conditions.find(*itr) == tempState->conditions.end()){
            goal = false;
        }
    }
    return goal;
} 
        
void arg_permutation(int cur, int numofArgs, vector<string> symbols, vector<string> level, vector<vector<string>> &perm, vector<bool>& visited){
    if (cur == numofArgs){
        perm.push_back(level);
        return;
    }
    else{
        for (int i = 0; i < symbols.size(); i++){
            if (!visited[i]){
                visited[i] = true;
                level.push_back(symbols[i]);
                arg_permutation(cur + 1, numofArgs, symbols, level, perm, visited);
                visited[i] = false;
                level.pop_back();
            }
        }

    }
}

// generate all possible next actions

list<GroundedAction> getAllActions(Env* env){
    list<GroundedAction> allActions;
    vector<string> symbols;
    for (auto symbol : env->get_symbols()){
        symbols.push_back(symbol);
    }
    int numofSymbols = symbols.size();

    GroundedCondition wrongEffect("Clear", {"Table"}, true);

    // for all possible action, generate grounded actions
    for (auto action : env->get_actions()){
        
        unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions = action.get_preconditions();
        unordered_set<Condition, ConditionHasher, ConditionComparator> effects = action.get_effects();

        string actionName = action.get_name();
        list<string> actionArgs = action.get_args();
        int numofArgs = action.get_args().size();

        // permutation of all symbols 
        vector<vector<string>> argPerm;
        vector<string> level;
        vector<bool> visited(numofSymbols, false);
        arg_permutation(0, numofArgs, symbols, level, argPerm, visited);                                 
        
        

        for (vector<string> tempArgs: argPerm){
            unordered_map<string, string> argsMap;
            auto action_iter = actionArgs.begin();
            for (int i = 0; i < numofArgs; ++i) {
                
                argsMap[*action_iter] = tempArgs[i];
                action_iter++;
            }

            list<string> groundactionArgs(tempArgs.begin(), tempArgs.end());
            GroundedAction groundAction(actionName, groundactionArgs);
            
            unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> gaPre;
            unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> gaEff;

            for (Condition precond: preconditions) {
                list<string> precond_args = precond.get_args();
                list<string> g_precond_args;
                for (string &precond_arg: precond_args) {
                    if (argsMap.find(precond_arg) != argsMap.end()) {
                        g_precond_args.push_back(argsMap[precond_arg]);
                    } else {
                        g_precond_args.push_back(precond_arg);
                    }
                }
                string predicate = precond.get_predicate();
                GroundedCondition g_precond(predicate, g_precond_args, true);
                gaPre.insert(g_precond);
            }

            for (Condition effect: effects) {
                list<string> effect_args = effect.get_args();
                list<string> g_effect_args;
                for (string &effect_arg: effect_args) {
                    if (argsMap.find(effect_arg) != argsMap.end()) {
                        g_effect_args.push_back(argsMap[effect_arg]);
                    } else {
                        g_effect_args.push_back(effect_arg);
                    }

                }
                string predicate = effect.get_predicate();
                bool g_truth = effect.get_truth();
                GroundedCondition g_effect(predicate, g_effect_args, true);
                g_effect.set_truth(g_truth);
                if (g_effect == wrongEffect) {
                    continue;
                }
                // if (g_truth){
                //     gaEff.insert(g_effect);
                // }
                gaEff.insert(g_effect);
            }

            groundAction.set_preconditions(gaPre);
            groundAction.set_effects(gaEff);
            allActions.push_back(groundAction);
        }
    }
    return allActions;
}


bool validGroundAction(State &tempState, GroundedAction &ga) {
    const unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> &gaPre = ga.get_preconditions();
    for (const GroundedCondition &tempGroundcondition: gaPre) {
        if (!tempState.correctCondition(tempGroundcondition)) {
            return false;
        }
    }
    return true;
}

void actionEffect(State *&currentState, State *&nextState, GroundedAction &g_action) {
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> currentNodeCoditions = currentState->get_conditions();
    for (GroundedCondition gc: currentNodeCoditions) {
        nextState->add_condition(gc);
    }
    for (GroundedCondition g_effect_cond: g_action.get_effects()) {
        if (g_effect_cond.get_truth()) {
            nextState->add_condition(g_effect_cond);
        } else {
            nextState->remove_condition(g_effect_cond);
        }
    }
}

bool checkCloseList(State* currentState, vector<State*> & closeList) {
    for (auto it = closeList.begin(); it != closeList.end(); it++) {
        if ((**it) == (*currentState)) {
            return true;
        }
    }
    return false;
}



list<GroundedAction> planner(Env* env)
{
    // this is where you insert your planner
    clock_t start, end1, end;
    start = clock();

    bool foundGoal = false;
    list<GroundedAction>  allActions = getAllActions(env);
    end1 = clock();
    float time_cost1 = (float) (end1 - start) / CLOCKS_PER_SEC;
    cout << "EXPLORE ALL ACTIONS DONE IN: " << time_cost1 << " s " << endl;
    // for (auto it : allActions){
    //     unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> eff = it.get_effects();
    //     cout << it.toString() << endl;
    //     for (auto itt: eff){
    //         cout << itt.toString() << endl;
    //     }
    //     cout <<  " " << endl;
    // }
    vector<State*> stateMap;
    
    priority_queue<State*, vector<State*>, Compare_State> openList;
    vector<State*> closeList;
    
    list<GroundedAction> actions;

    State* initState = new State(env->get_initConditions());
    State* goalState = new State(env->get_goalConditions());

    initState->setGval(0.0);
    double h_temp = calc_Heuristic(initState, goalState);
    initState->setHval(h_temp);
    initState->updateFval();

    openList.push(initState);
    stateMap.push_back(initState);

    while(!openList.empty()){
        State* currentState = openList.top();
        openList.pop();
        if(at_goal(currentState, goalState)){
            cout << "GOAL FOUND" << endl;
            goalState = currentState;
            foundGoal = true;
            break;
        }
        closeList.push_back(currentState);
        for (GroundedAction& nextAction: allActions) {
            if (validGroundAction(*currentState, nextAction)) {
                State *nextState = new State();
                nextState->action = &nextAction;
                nextState->parent = currentState;
                actionEffect(currentState, nextState, nextAction);

                if (checkCloseList(nextState, closeList)) {
                    continue;
                }

                for (auto it = stateMap.begin(); it != stateMap.end(); it++) {
                    if (**it == *nextState) {
                        nextState = *it;
                        if (nextState->g > currentState->g + 1) {
                            nextState->g = currentState->g + 1;
                            nextState->parent = currentState;
                            nextState->action = &nextAction;
                             
                        }
                        goto skip;
                        
                    }
                }
                nextState->g = currentState->g + 1;
                nextState->h = calc_Heuristic(nextState, goalState);

                openList.push(nextState);
                stateMap.push_back(nextState);
                skip:
                nextState->updateFval(); 
                
            }
        }

    }
    
    if (foundGoal){
        int totalStatesize = closeList.size() + stateMap.size();
        cout << "TOTAL STATES EXPANDED: " << totalStatesize << endl;
        State *traceState = goalState;
        while (traceState->parent != nullptr) {
            GroundedAction* actionPtr = traceState->action;
            if (actionPtr == nullptr){
                cout << "TF??" << endl;
            }
            actions.push_front(*(traceState->action));
            traceState = traceState->parent;
        }
    }else{
        cout << "GOAL NOT FOUND" << endl;
    }

    end = clock();
    float time_cost = (float) (end - start) / CLOCKS_PER_SEC;
    cout << "PLANNING DONE IN: " << time_cost << " s " << endl;


    // blocks world example
    // actions.push_back(GroundedAction("MoveToTable", { "A", "B" }));
    // actions.push_back(GroundedAction("Move", { "C", "Table", "A" }));
    // actions.push_back(GroundedAction("Move", { "B", "Table", "C" }));

    return actions;
}

int main(int argc, char* argv[])
{
    // DO NOT CHANGE THIS FUNCTION
    char* filename = (char*)("example.txt");
    if (argc > 1)
        filename = argv[1];

    cout << "Environment: " << filename << endl << endl;
    Env* env = create_env(filename);
    if (print_status)
    {
        cout << *env;
    }

    list<GroundedAction> actions = planner(env);

    cout << "\nPlan: " << endl;
    for (GroundedAction gac : actions)
    {
        cout << gac << endl;
    }

    return 0;
}