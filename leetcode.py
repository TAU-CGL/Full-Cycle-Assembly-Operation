from collections import deque


class Solution(object):
    def isValid(self, s):
        """
        :type s: str
        :rtype: bool
        """
        parenthesesDict = {"}": "{", "]": "[", ")": "("}
        stack = deque([])
        for i in s:
            if i in "{([":
                stack.append(i)
            elif len(stack) == 0:
                return False
            else:
                lastOpen = stack.pop()
                if lastOpen != i:
                    return False
        return len(stack) == 0


sol = Solution()
print(sol.isValid("a"))
