# Project 03 Questions

## Name

- Name: Todd Nguyen
- ID: noid

## Question 1

_Q1. What was the Spaghetti Factor of the module with the highest SF? If any modules have an SF greater than 10, why? (Note that a "module" is a single procedure or subroutine, and each module has its own SF number. It is NOT necessarily all the code in a single .c file.) Only count global variables referenced by the specific module when determinine that module's SF, even if there are other globals in the file._

The Spaghetti Factor with the highest factor was about 15, as there were quite a few modules with copied pasted code.

## Question 2

_Q2. List the numbers of the checklist items that you had already attempted on Project 2._

I have attempted these steps before this project:

- Num 1: All the code was in a single cpp file.
- Num 2: ALl indentation, variable naming and organizational style is consistent. I used VS Code's C++ autoformat feature using Google's formatting to do this.
- Num 4: Spaces used instead of tabs. I have always used Spaces with either 2 or 4 spaces whenever I code.
- Num 5: Variable name has meaning; I did this thanks to the Clean Code training.
- Num 6: Variables and procedures have minimum scope. I used "class" variables whenever necessary; otherwise I had it so that each variable is as local as possible.
- Num 7: Most variables are automatic. I have no static variables at all.
- Num 13: "Magic numbers" are not used. I try to use `const` to list constants whenever possible.
- Num 14: `switch` statements are used to decide among enum values. Enum values are basically made for `switch` statements, so I try to use `switch` statements with enums whenever possible.
- Num 17: No copy-pasted code is present. I try to factor out common functions whenever possible to reduce the amount of copy-pasted code and use functions instead.
- Num 19: No bit-wise operations are used for math. Since the only mathematical functions in our code was increment / decrement by 1, no bit-wise operations were used. I will keep this in mind to not use bit-wise operations in the future.
- Num 20: Math is not performed on enum values. The enum values that I used were only for assignments or for `case` statements.

## Q3

_Q3. Do you strongly disagree with any items on the checklist? Which ones, and why?_

The only item I strongly disagree with is 16: All conditionally executed sets of statements are enclosed by "{}". I think that for one line statements you don't necessary need "{}". However, I do see the merit behind having "{}" after conditionals, as it does make it clearer which block is executed after the conditional.

## Q4

_Q4. Is there anything important you think should be added to the checklist? Why?_

I think an explicit format guide should be followed, e.g. either Google's style-guide or Visual Studio's style guide.

## Q5

_Q5. You are permitted to not fix up to three checklist items. List zero through three items you skipped. ("none" if you didn't skip)_

None.

## Q6

_Q6. Would you change anything about Project 3? If so, what?_

Nothing, project 03 was a good explanation on refactoring!
