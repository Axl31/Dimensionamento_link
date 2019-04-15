function plot_ottimizza_link(somma_link, differenza_link, cost_function, links_sum_norm, links_diff_norm)

    figure()
    plot(somma_link,'-r','LineWidth',3)
    hold on
    plot(differenza_link,'-b','LineWidth',3)

    figure()
    plot(cost_function,'-g','LineWidth',3)
    hold on
    plot(links_sum_norm,'-r','LineWidth',3)
    hold on
    plot(links_diff_norm,'-b','LineWidth',3)

end