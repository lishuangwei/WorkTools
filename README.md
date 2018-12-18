# WorkToolsfs
first pc install git lfs

// Skip smudge - We'll download binary files later in a faster batch
git lfs install --skip-smudge

// Do git clone here
git clone ...

// Fetch all the binary files in the new clone
git lfs pull

// Reinstate smudge
git lfs install --force
