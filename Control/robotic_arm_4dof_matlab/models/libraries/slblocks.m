function blkStruct = slblocks
    % This function specifies that the library 'MyLibraryName' 
    % should appear in the Library Browser with the name 'My Custom Tools'

    % 1. Define the Library File Name (without .slx extension)
    Browser.Library = 'BUMBLEBEE';

    % 2. Define the Display Name in the Browser tree
    Browser.Name = 'BumbleBee';

    % 3. (Optional) Define if the library should be at the top level
    % IsFlat = 0 means it has a tree structure; IsFlat = 1 is flat.
    Browser.IsFlat = 0; 

    blkStruct.Browser = Browser;
end
