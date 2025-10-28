PyTorch Note
===

## Tensor operations

`torch.zeros()`
`torch.ones()`
`torch.rand()`

`torch.arange()`

`torch.meshgrid()`

`torch.tensor.flatten()`

> '_' at the end of a method in PyTorch indicates that this method operates in-place.

`torch.empty(1).uniform_(-1, 1)`

`torch.tensor.unsqueeze()` 

`torch.any()`

`torch.norm()`

`torch.cat()`: 

`torch.stack()`: 

## Contiguous

A tensor is **contiguous** if its elements are stored in a single, contiguous block of memory. 

**Strides** are the number of elements to skip in each dimension when traversing the tensor.

## Slicing

**Basic slicing**. The resulting tensor is a view of the original tensor.

```py
x = torch.ones(4, 4)
b = x[1:3, 1:3]  # Slicing rows and columns
x[1:3, 1:3] = 0  # Setting a slice to zero
```

Advance slicing. Boolean indexing, non-contiguous indexing, etc.

```py
x = torch.ones(4, 4)

x[[0, 2, 3], 1] = 0  # Setting specific rows and a column to zero
x[[0, 2, 3]][1] = 0  # Does not change the original tensor, since this is a conbined brackets
x[[0, 2, 3], [1, 2, 0]] = 0  # Setting specific elements using advanced indexing, not the selected rows and cols
```
